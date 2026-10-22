#include "shaoshao_runtime/serial.hpp"
#include "shaoshao_runtime/port.hpp"
#include "shaoshao_runtime/http.hpp"
#include <drogon/drogon.h>
#include <iostream>
#include <mutex>
#include <thread>
#include <sys/stat.h>
#ifdef SHAOSHAO_RUNTIME_ROS
#include "shaoshao_runtime/ros.hpp"
#include <shaoshao_hw_interfaces/msg/nav_command.hpp>
#endif

using namespace hw_runtime;
int main(int argc,char** argv){
  try{
    fs::path root=runtime_path(),config,robot;std::string bind="0.0.0.0";int port=8081;bool ros=false;
    for(int i=1;i<argc;++i){std::string key=argv[i];if(key=="--hw-ros"){ros=true;continue;}if(i+1>=argc)throw std::invalid_argument("Missing serial service argument");std::string value=argv[++i];if(key=="--runtime")root=value;else if(key=="--config")config=value;else if(key=="--robot-config")robot=value;else if(key=="--bind")bind=value;else if(key=="--port")port=std::stoi(value);else throw std::invalid_argument("Unknown serial service argument");}
    if(port<1||port>65535)throw std::invalid_argument("Invalid manual service port");if(config.empty())config=root/"config/serial.navcar.yaml";if(robot.empty())robot=root/"shaoshao/web_panel/robot.yaml";
    serial::Settings settings(config);if(fs::is_regular_file(robot)){Config vehicle(robot);settings.max_speed=std::min(settings.max_speed,vehicle.speed);}
    serial::Controller controller(settings);std::mutex mutex;Port uart(settings.port);
    fs::create_directories(root);std::string token;if(fs::exists(root/"navcar_move.token")){token=read_text(root/"navcar_move.token",1024);while(!token.empty()&&std::isspace(static_cast<unsigned char>(token.back())))token.pop_back();}else{token=random_token();atomic_write(root/"navcar_move.token",token+"\n");chmod((root/"navcar_move.token").c_str(),0600);}if(token.empty())throw std::runtime_error("Manual access token is empty");
    atomic_write(root/"move_web.pid",std::to_string(getpid())+"\n");
    std::jthread worker([&](std::stop_token stop){
      try{while(!stop.stop_requested()){
        auto bytes=uart.read();double now=steady();{std::lock_guard<std::mutex> lock(mutex);controller.receive(bytes,now);if(controller.armed&&controller.session.error.empty()){
          auto target=controller.desired(now);bool moving_pending=false;
          if(!controller.session.pending.empty()){auto previous=serial::values(controller.session.pending);moving_pending=previous[0]||previous[1]||previous[2];}
          if(moving_pending&&target[0]==0&&target[1]==0&&target[2]==0&&now-controller.session.sent_at>=.05)controller.session.fail("运动指令 ACK 超时，停止发帧；请重启控制服务");
          else{auto packet=controller.session.poll(target,now);if(!packet.empty())uart.write(packet);}
        }}std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }}catch(const std::exception& e){std::lock_guard<std::mutex> lock(mutex);controller.session.fail(e.what());}
      {std::lock_guard<std::mutex> lock(mutex);controller.action="stop";controller.updated=0;controller.spin_enabled=false;}
      double until=steady()+.25;
      while(steady()<until){try{std::lock_guard<std::mutex> lock(mutex);if(!controller.armed||!controller.session.error.empty())break;controller.receive(uart.read(),steady());auto pending=controller.session.pending;if(!pending.empty()){auto target=serial::values(pending);if((target[0]||target[1]||target[2])&&steady()-controller.session.sent_at>=.05)break;}auto packet=controller.session.poll(controller.desired(steady()),steady());if(!packet.empty())uart.write(packet);}catch(const std::exception&){break;}std::this_thread::sleep_for(std::chrono::milliseconds(2));}
    });
#ifdef SHAOSHAO_RUNTIME_ROS
    rclcpp::Node::SharedPtr node;std::unique_ptr<SpinThread> executor;rclcpp::Subscription<shaoshao_hw_interfaces::msg::NavCommand>::SharedPtr subscription;
    if(ros){rclcpp::init(0,nullptr,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);node=std::make_shared<rclcpp::Node>("shaoshao_hw_serial");
      subscription=node->create_subscription<shaoshao_hw_interfaces::msg::NavCommand>("/hw/nav_command",1,[&](shaoshao_hw_interfaces::msg::NavCommand::SharedPtr msg){
        std::lock_guard<std::mutex> lock(mutex);try{const auto& v=msg->twist;if(!fresh(node->now().seconds(),msg->header.stamp,.2)||msg->header.frame_id!="navcar_center"||v.linear.z!=0||v.angular.x!=0||v.angular.y!=0||v.angular.z!=0)throw std::invalid_argument("Invalid/stale leased velocity");Json::Value data;data["lease"]=msg->lease;data["seq"]=Json::UInt64(msg->seq);data["forward"]=v.linear.x;data["left"]=v.linear.y;controller.navigation_command(data,steady());}
        catch(const std::exception&){if(msg->lease==controller.lease){controller.action="stop";controller.navigation.setZero();controller.updated=0;}}
      });executor=std::make_unique<SpinThread>(node);}
#else
    if(ros)throw std::runtime_error("This serial service was built without ROS support");
#endif
    auto reply=[](int code,const Json::Value& data){auto out=drogon::HttpResponse::newHttpJsonResponse(data);out->setStatusCode(static_cast<drogon::HttpStatusCode>(code));out->addHeader("Cache-Control","no-store");out->addHeader("X-Content-Type-Options","nosniff");return out;};
    auto& app=drogon::app();app.setThreadNum(2).setFileTypes({}).setDocumentRoot((root/"manual-static-disabled").string()).setUploadPath((root/"manual-uploads").string()).setClientMaxBodySize(1024).setLogLevel(trantor::Logger::kWarn).addListener(bind,static_cast<uint16_t>(port));
    app.registerHandler("/",[root](const drogon::HttpRequestPtr&,std::function<void(const drogon::HttpResponsePtr&)>&& callback){callback(drogon::HttpResponse::newFileResponse((root/"shaoshao/web_panel/navcar_move.html").string()));},{drogon::Get});
    app.registerHandler("/api/state",[&](const drogon::HttpRequestPtr& request,std::function<void(const drogon::HttpResponsePtr&)>&& callback){Json::Value value;if(!equal_token(request->getHeader("authorization"),"Bearer "+token)){value["error"]="访问令牌错误";callback(reply(401,value));return;}std::lock_guard<std::mutex> lock(mutex);callback(reply(200,controller.state(steady())));},{drogon::Get});
    for(const std::string route:{"/api/command","/api/yaw","/api/spin","/api/navigation/acquire","/api/navigation/command","/api/navigation/release"}){
      app.registerHandler(route,[&,route](const drogon::HttpRequestPtr& request,std::function<void(const drogon::HttpResponsePtr&)>&& callback){
        Json::Value output;if(!equal_token(request->getHeader("authorization"),"Bearer "+token)){output["error"]="访问令牌错误";callback(reply(401,output));return;}
        try{auto data=json_parse(std::string(request->body()));if(!data.isObject())throw std::invalid_argument("需要命令对象");std::lock_guard<std::mutex> lock(mutex);double now=steady();
          if(route=="/api/navigation/acquire")output["lease"]=controller.acquire(now);
          else{if(route=="/api/command")controller.manual(data,now);else if(route=="/api/yaw")controller.set_yaw(data,now);else if(route=="/api/spin")controller.set_spin(data,now);else if(route=="/api/navigation/command")controller.navigation_command(data,now);else controller.release(data["lease"].asString());output=controller.state(now);}callback(reply(200,output));
        }catch(const std::invalid_argument& e){output["error"]=e.what();callback(reply(400,output));}catch(const std::exception& e){output["error"]=e.what();callback(reply(409,output));}
      },{drogon::Post});
    }
    std::cout<<"NAVCAR MOVE: http://<vehicle-ip>:"<<port<<"/?token="<<token<<std::endl;app.run();
#ifdef SHAOSHAO_RUNTIME_ROS
    executor.reset();if(ros)rclcpp::shutdown();
#endif
    worker.request_stop();worker.join();return 0;
  }catch(const std::exception& e){std::cerr<<"Native serial service failed: "<<e.what()<<'\n';return 1;}
}
