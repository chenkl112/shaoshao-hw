#pragma once
#include <json/json.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

namespace shaoshao_web {
namespace fs=std::filesystem;
using Clock=std::chrono::steady_clock;
inline std::string json(const Json::Value& value){Json::StreamWriterBuilder writer;writer["indentation"]="";return Json::writeString(writer,value);}
inline Json::Value parse(const std::string& text){
  Json::CharReaderBuilder builder;builder["failIfExtra"]=true;builder["rejectDupKeys"]=true;
  auto reader=std::unique_ptr<Json::CharReader>(builder.newCharReader());Json::Value value;std::string error;
  if(!reader->parse(text.data(),text.data()+text.size(),&value,&error))throw std::invalid_argument("Invalid JSON");
  return value;
}
inline bool token_equal(const std::string& a,const std::string& b){
  size_t different=a.size()^b.size();for(size_t i=0;i<std::max(a.size(),b.size());++i)different|=(i<a.size()?static_cast<unsigned char>(a[i]):0)^(i<b.size()?static_cast<unsigned char>(b[i]):0);
  return different==0;
}
inline Json::Value rpc(const fs::path& socket_path,const Json::Value& request){
  struct Socket {int fd=-1;~Socket(){if(fd>=0)::close(fd);}} socket;
  socket.fd=::socket(AF_UNIX,SOCK_STREAM|SOCK_CLOEXEC,0);if(socket.fd<0)throw std::runtime_error("Cannot create supervisor socket");
  timeval timeout{0,500000};setsockopt(socket.fd,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));setsockopt(socket.fd,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout));
  sockaddr_un address{};address.sun_family=AF_UNIX;auto name=socket_path.string();
  if(name.size()>=sizeof(address.sun_path))throw std::invalid_argument("Supervisor socket path too long");
  std::memcpy(address.sun_path,name.c_str(),name.size()+1);
  if(connect(socket.fd,reinterpret_cast<sockaddr*>(&address),sizeof(address)))throw std::runtime_error("Supervisor unavailable");
  std::string payload=json(request)+"\n";size_t sent=0;
  while(sent<payload.size()){
    auto n=send(socket.fd,payload.data()+sent,payload.size()-sent,MSG_NOSIGNAL);if(n<=0)throw std::runtime_error("Supervisor send failed");sent+=static_cast<size_t>(n);
  }
  std::string response;std::array<char,16384> buffer;
  auto deadline=Clock::now()+std::chrono::milliseconds(800);
  while(response.size()<16*1024*1024&&Clock::now()<deadline){
    auto n=recv(socket.fd,buffer.data(),buffer.size(),0);if(n<=0)throw std::runtime_error("Supervisor response interrupted");
    response.append(buffer.data(),static_cast<size_t>(n));
    auto end=response.find('\n');if(end!=std::string::npos)return parse(response.substr(0,end));
  }
  throw std::runtime_error("Supervisor response limit exceeded");
}

struct Cloud {
  uint64_t sequence=0;std::string frame,source;std::vector<std::array<float,3>> points;
  Clock::time_point received{};
};
inline std::string binary_cloud(const Cloud& cloud){
  Json::Value header;header["sequence"]=Json::UInt64(cloud.sequence);header["frame"]=cloud.frame;header["source"]=cloud.source;header["count"]=static_cast<unsigned>(cloud.points.size());
  auto metadata=json(header);std::string out="SCPC";
  auto append=[&](uint32_t value){for(int i=0;i<4;++i)out.push_back(static_cast<char>((value>>(8*i))&255));};
  append(static_cast<uint32_t>(metadata.size()));out+=metadata;
  for(const auto& p:cloud.points)for(float value:p)append(std::bit_cast<uint32_t>(value));
  return out;
}
inline Json::Value cloud_json(const Cloud& cloud){
  Json::Value out;out["sequence"]=Json::UInt64(cloud.sequence);out["frame"]=cloud.frame;out["source"]=cloud.source;out["points"]=Json::Value(Json::arrayValue);
  for(const auto& p:cloud.points){Json::Value point(Json::arrayValue);for(float value:p)point.append(value);out["points"].append(point);}return out;
}

class Backend {
public:
  Backend(fs::path socket,fs::path maps_root,bool fixture=false):socket_(std::move(socket)),maps_root_(fs::weakly_canonical(maps_root)),fixture_(fixture){
    worker_=std::jthread([this](std::stop_token stop){
      unsigned cycle=0;
      while(!stop.stop_requested()){
        try{
          Json::Value request;request["operation"]="snapshot";auto reply=rpc(socket_,request);
          if(reply["status"].asInt()!=200||!reply["body"]["state"].isObject())throw std::runtime_error("Invalid supervisor snapshot");
          auto mode=reply["body"]["state"]["mode"].asString();
          if(mode!="idle"&&mode!="mapping"&&mode!="navigation")throw std::runtime_error("Invalid supervisor mode");
          {
            std::lock_guard<std::mutex> lock(mutex_);snapshot_=reply["body"];updated_=Clock::now();
            active_=snapshot_["state"]["mode"].asString()!="idle";
            if(fixture_&&snapshot_["cloud"].isObject())fixture_cloud(snapshot_["cloud"]);
          }
          if(cycle++%5==0){request["operation"]="costmap";auto cost=rpc(socket_,request);if(cost["status"].asInt()==200){std::lock_guard<std::mutex> lock(mutex_);costmap_=cost["body"];}}
        }catch(const std::exception&){}
        for(int i=0;i<10&&!stop.stop_requested();++i)std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    });
  }
  ~Backend(){worker_.request_stop();if(worker_.joinable())worker_.join();}
  Json::Value snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if(Clock::now()-updated_>std::chrono::seconds(1))throw std::runtime_error("任务服务连接已中断");
    auto out=snapshot_;out["state"]["cloud_age"]=cloud_.sequence&&!cloud_.points.empty()?Json::Value(std::chrono::duration<double>(Clock::now()-cloud_.received).count()):Json::Value();
    out["state"]["transport"]["websocket"]=true;out["state"]["transport"]["binary_cloud"]=true;
    return out;
  }
  Json::Value costmap() const {std::lock_guard<std::mutex> lock(mutex_);return costmap_;}
  Cloud cloud() const {std::lock_guard<std::mutex> lock(mutex_);return cloud_;}
  bool active() const{return active_.load();}
  void update_cloud(std::vector<std::array<float,3>> points,std::string frame,std::string source){
    if(points.size()>6000)throw std::invalid_argument("Cloud display limit exceeded");
    for(const auto& p:points)for(float value:p)if(!std::isfinite(value))throw std::invalid_argument("Nonfinite display point");
    std::lock_guard<std::mutex> lock(mutex_);++cloud_.sequence;cloud_.points=std::move(points);cloud_.frame=std::move(frame);cloud_.source=std::move(source);cloud_.received=Clock::now();
  }
  Json::Value command(const std::string& path,const Json::Value& body){Json::Value request;request["operation"]="command";request["path"]=path;request["body"]=body;return rpc(socket_,request);}
  fs::path image(const std::string& revision) const {
    auto snapshot=this->snapshot();if(!snapshot["map"]["available"].asBool())throw std::runtime_error("暂无地图");
    if(!revision.empty()&&revision!=snapshot["map"]["revision"].asString())throw std::runtime_error("地图已更新，请刷新");
    auto path=fs::weakly_canonical(snapshot["image_path"].asString());auto relative=path.lexically_relative(maps_root_);
    if(relative.empty()||relative.is_absolute()||*relative.begin()==".."||path.extension()!=".png"||!fs::is_regular_file(path)||fs::file_size(path)>32*1024*1024)
      throw std::runtime_error("地图图像不可用");
    return path;
  }
private:
  fs::path socket_,maps_root_;bool fixture_;mutable std::mutex mutex_;
  Json::Value snapshot_,costmap_=Json::Value(Json::objectValue);Cloud cloud_;
  Clock::time_point updated_{};std::atomic<bool> active_{false};std::jthread worker_;
  void fixture_cloud(const Json::Value& value){
    uint64_t sequence=value["sequence"].asUInt64();if(sequence==cloud_.sequence)return;
    const auto& list=value["points"];if(!list.isArray()||list.size()>6000)throw std::runtime_error("Invalid fixture cloud");
    Cloud next;next.sequence=sequence;next.frame=value["frame"].asString();next.source="fixture";next.received=Clock::now();
    for(const auto& point:list){if(!point.isArray()||point.size()!=3)throw std::runtime_error("Invalid fixture point");std::array<float,3> p;
      for(int i=0;i<3;++i){p[i]=point[i].asFloat();if(!std::isfinite(p[i]))throw std::runtime_error("Nonfinite fixture point");}next.points.push_back(p);}
    cloud_=std::move(next);
  }
};
}
