#include "shaoshao_web/backend.hpp"
#include <drogon/drogon.h>
#include <drogon/WebSocketController.h>
#include <trantor/utils/ConcurrentTaskQueue.h>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <set>

#ifdef SHAOSHAO_WEB_ROS
namespace shaoshao_web {std::function<void()> start_telemetry(std::shared_ptr<Backend>);}
#endif

using namespace shaoshao_web;
using Reply=std::function<void(const drogon::HttpResponsePtr&)>;
namespace {
struct Cleanup {std::function<void()> stop;~Cleanup(){if(stop)stop();}};
const std::set<std::string> commands{"/api/mode","/api/spin","/api/spin/heartbeat","/api/navigation/goal",
  "/api/navigation/plan","/api/navigation/cancel","/api/mapping/filter","/api/localization/retry"};
void headers(const drogon::HttpResponsePtr& response){
  response->addHeader("Cache-Control","no-store");response->addHeader("X-Content-Type-Options","nosniff");response->addHeader("Referrer-Policy","no-referrer");
}
drogon::HttpResponsePtr response(int status,const Json::Value& body){
  auto out=drogon::HttpResponse::newHttpJsonResponse(body);out->setStatusCode(static_cast<drogon::HttpStatusCode>(status));headers(out);return out;
}
drogon::HttpResponsePtr error(int status,const std::string& reason){Json::Value body;body["error"]=reason;return response(status,body);}
bool authorized(const drogon::HttpRequestPtr& request,const std::string& token){return token_equal(request->getHeader("authorization"),"Bearer "+token);}

struct Client {
  std::weak_ptr<drogon::WebSocketConnection> connection;
  std::atomic<bool> authenticated{false},waiting{false};std::atomic<uint64_t> serial{0};
  uint64_t cloud_sequence=std::numeric_limits<uint64_t>::max();
  Clock::time_point created=Clock::now(),sent=Clock::now();
};
class Stream final:public drogon::WebSocketController<Stream,false>{
public:
  Stream(std::shared_ptr<Backend> backend,std::string token):backend_(std::move(backend)),token_(std::move(token)){}
  WS_PATH_LIST_BEGIN
  WS_PATH_ADD("/ws");
  WS_PATH_LIST_END
  void handleNewConnection(const drogon::HttpRequestPtr&,const drogon::WebSocketConnectionPtr& connection) override {
    std::lock_guard<std::mutex> lock(mutex_);
    std::erase_if(clients_,[](const auto& client){auto c=client->connection.lock();return !c||!c->connected();});
    if(clients_.size()>=8){connection->shutdown(drogon::CloseCode::kViolation,"Connection limit");return;}
    auto client=std::make_shared<Client>();client->connection=connection;connection->setContext(client);clients_.push_back(client);
  }
  void handleConnectionClosed(const drogon::WebSocketConnectionPtr&) override {}
  void handleNewMessage(const drogon::WebSocketConnectionPtr& connection,std::string&& text,const drogon::WebSocketMessageType& type) override {
    if(type==drogon::WebSocketMessageType::Ping||type==drogon::WebSocketMessageType::Pong)return;
    auto client=connection->getContext<Client>();
    if(!client||type!=drogon::WebSocketMessageType::Text||text.size()>1024){connection->shutdown(drogon::CloseCode::kViolation,"Invalid message");return;}
    try{
      auto message=parse(text);if(!message.isObject())throw std::invalid_argument("Expected object");
      if(!client->authenticated){
        if(message["type"].asString()!="auth"||!message["token"].isString()||!token_equal(message["token"].asString(),token_))throw std::invalid_argument("Invalid token");
        client->authenticated=true;connection->send("{\"type\":\"ready\"}");
      }else if(message["type"].asString()=="ack"&&message["serial"].isUInt64()&&message["serial"].asUInt64()==client->serial){
        client->waiting=false;
      }else throw std::invalid_argument("Unexpected message");
    }catch(const std::exception&){connection->shutdown(drogon::CloseCode::kViolation,"Authentication or message failed");}
  }
  void broadcast(){
    std::lock_guard<std::mutex> lock(mutex_);auto now=Clock::now();
    std::erase_if(clients_,[](const auto& client){auto c=client->connection.lock();return !c||!c->connected();});
    for(const auto& client:clients_){
      auto connection=client->connection.lock();if(!connection)continue;
      if(!client->authenticated){if(now-client->created>std::chrono::seconds(3))connection->forceClose();continue;}
      if(client->waiting){if(now-client->sent>std::chrono::seconds(3))connection->forceClose();continue;}
      Json::Value message;message["serial"]=Json::UInt64(++client->serial);message["type"]="state";
      Cloud cloud;bool send_cloud=false;
      try{
        auto snapshot=backend_->snapshot();cloud=backend_->cloud();send_cloud=cloud.sequence!=client->cloud_sequence;
        message["body"]=snapshot["state"];message["cloud"]=send_cloud;message["cloud_sequence"]=Json::UInt64(cloud.sequence);
      }catch(const std::exception&){message["type"]="unavailable";message["error"]="任务服务连接已中断";}
      client->waiting=true;client->sent=now;connection->send(json(message));
      if(send_cloud){connection->send(binary_cloud(cloud),drogon::WebSocketMessageType::Binary);client->cloud_sequence=cloud.sequence;}
    }
  }
private:
  std::shared_ptr<Backend> backend_;std::string token_;std::mutex mutex_;std::vector<std::shared_ptr<Client>> clients_;
};
}

int main(int argc,char** argv){
  try{
    fs::path socket,assets,maps_root;std::string bind="127.0.0.1";int port=8080;bool fixture=false;
    for(int i=1;i<argc;++i){std::string option=argv[i];
      if(option=="--fixture"){fixture=true;continue;}
      if(i+1>=argc)throw std::invalid_argument("Missing option value");std::string value=argv[++i];
      if(option=="--socket")socket=value;else if(option=="--assets")assets=value;else if(option=="--maps-root")maps_root=value;
      else if(option=="--bind")bind=value;else if(option=="--port")port=std::stoi(value);else throw std::invalid_argument("Unknown option");
    }
    if(socket.empty()||assets.empty()||maps_root.empty()||port<1||port>65535||!fs::is_regular_file(assets/"index.html")||!fs::is_regular_file(assets/"stream.js"))
      throw std::invalid_argument("Provide --socket, --assets and --maps-root with valid assets/port");
    const char* environment=std::getenv("SCURM_WEB_TOKEN");std::string token=environment?environment:"";
    if(token.empty()||token.size()>256)throw std::invalid_argument("SCURM_WEB_TOKEN must contain the access token");
#ifndef SHAOSHAO_WEB_ROS
    if(!fixture)throw std::invalid_argument("This build requires --fixture; build with ROS for vehicle telemetry");
#else
    if(fixture)throw std::invalid_argument("Fixture mode is only available in the standalone validation build");
#endif
    auto backend=std::make_shared<Backend>(socket,maps_root,fixture);
#ifdef SHAOSHAO_WEB_ROS
    Cleanup telemetry{start_telemetry(backend)};
#endif
    auto stream=std::make_shared<Stream>(backend,token);
    auto tasks=std::make_shared<trantor::ConcurrentTaskQueue>(1,"web_commands");
    auto pending=std::make_shared<std::atomic<unsigned>>(0);
    auto static_root=socket.parent_path()/"web-static";
    auto upload_root=socket.parent_path()/"web-uploads";
    fs::create_directories(static_root);fs::create_directories(upload_root);
    fs::permissions(static_root,fs::perms::owner_all,fs::perm_options::replace);
    fs::permissions(upload_root,fs::perms::owner_all,fs::perm_options::replace);
    auto& app=drogon::app();app.setThreadNum(2).setMaxConnectionNum(64).setMaxConnectionNumPerIP(16)
      .setClientMaxBodySize(1024).setClientMaxWebSocketMessageSize(1024).setIdleConnectionTimeout(15)
      .setFileTypes({}).setDocumentRoot(static_root.string()).setUploadPath(upload_root.string())
      .setLogLevel(trantor::Logger::kWarn).enableGzip(true).enableBrotli(false).addListener(bind,static_cast<uint16_t>(port));
    app.registerPostHandlingAdvice([](const drogon::HttpRequestPtr&,const drogon::HttpResponsePtr& reply){headers(reply);});
    for(const std::string path:{"/","/stream.js"}){
      app.registerHandler(path,[assets,path](const drogon::HttpRequestPtr&,Reply&& callback){
        auto file=assets/(path=="/"?"index.html":"stream.js");auto reply=drogon::HttpResponse::newFileResponse(file.string());
        reply->setContentTypeCode(path=="/"?drogon::CT_TEXT_HTML:drogon::CT_TEXT_JAVASCRIPT);headers(reply);callback(reply);
      },{drogon::Get});
    }
    for(const std::string path:{"/api/state","/api/cloud","/api/map","/api/map/image","/api/navigation/costmap"}){
      app.registerHandler(path,[backend,token,path](const drogon::HttpRequestPtr& request,Reply&& callback){
        if(!authorized(request,token)){callback(error(401,"Invalid access token"));return;}
        try{
          auto snapshot=backend->snapshot();
          if(path=="/api/map/image"){
            auto reply=drogon::HttpResponse::newFileResponse(backend->image(request->getParameter("revision")).string());headers(reply);callback(reply);
          }else callback(response(200,path=="/api/state"?snapshot["state"]:path=="/api/map"?snapshot["map"]:
              path=="/api/cloud"?cloud_json(backend->cloud()):backend->costmap()));
        }catch(const std::exception& e){callback(error(path=="/api/map/image"?409:503,e.what()));}
      },{drogon::Get});
    }
    for(const auto& path:commands){
      app.registerHandler(path,[backend,token,path,tasks,pending](const drogon::HttpRequestPtr& request,Reply&& callback){
        if(!authorized(request,token)){callback(error(401,"Invalid access token"));return;}
        Json::Value body;
        try{body=parse(std::string(request->body()));if(!body.isObject())throw std::invalid_argument("Expected JSON object");}
        catch(const std::exception&){callback(error(400,"Invalid command JSON"));return;}
        if(pending->fetch_add(1)>=16){--*pending;callback(error(429,"Too many pending requests"));return;}
        auto queued=Clock::now();
        tasks->runTaskInQueue([backend,path,body,callback=std::move(callback),pending,queued]{
          try{
            if(Clock::now()-queued>std::chrono::milliseconds(500)){callback(error(503,"Command expired before dispatch"));--*pending;return;}
            auto result=backend->command(path,body);int code=result["status"].asInt();callback(response(code==200?202:code,result["body"]));
          }
          catch(const std::exception& e){callback(error(503,e.what()));}
          --*pending;
        });
      },{drogon::Post});
    }
    app.registerController(stream);app.getLoop()->runEvery(.2,[stream]{stream->broadcast();});
    std::cout<<"C++ WEB: http://"<<bind<<":"<<port<<"/"<<std::endl;
    app.run();
    return 0;
  }catch(const std::exception& e){std::cerr<<"Web startup failed: "<<e.what()<<std::endl;return 1;}
}
