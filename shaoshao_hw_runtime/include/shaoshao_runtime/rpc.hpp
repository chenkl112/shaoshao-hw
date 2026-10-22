#pragma once
#include "common.hpp"
#include <atomic>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <functional>
#include <mutex>
#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <thread>

namespace hw_runtime {
inline Json::Value rpc_call(const fs::path& path,const Json::Value& request){
  struct Socket{int fd=-1;~Socket(){if(fd>=0)close(fd);}} socket;
  socket.fd=::socket(AF_UNIX,SOCK_STREAM|SOCK_CLOEXEC,0);sockaddr_un address{};address.sun_family=AF_UNIX;auto name=path.string();if(name.size()>=sizeof(address.sun_path))throw std::invalid_argument("Supervisor socket path too long");std::memcpy(address.sun_path,name.c_str(),name.size()+1);
  if(socket.fd<0||connect(socket.fd,reinterpret_cast<sockaddr*>(&address),sizeof(address)))throw std::runtime_error("Native supervisor unavailable");timeval timeout{1,0};setsockopt(socket.fd,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));setsockopt(socket.fd,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout));
  auto data=json_text(request)+"\n";size_t offset=0;while(offset<data.size()){auto n=send(socket.fd,data.data()+offset,data.size()-offset,MSG_NOSIGNAL);if(n<=0)throw std::runtime_error("Supervisor send failed");offset+=static_cast<size_t>(n);}
  std::string reply;std::array<char,16384> buffer;double deadline=steady()+2;
  while(reply.size()<16*1024*1024&&steady()<deadline){auto n=recv(socket.fd,buffer.data(),buffer.size(),0);if(n<=0)throw std::runtime_error("Supervisor response interrupted");reply.append(buffer.data(),static_cast<size_t>(n));auto end=reply.find('\n');if(end!=std::string::npos){auto value=json_parse(reply.substr(0,end));if(value["status"].asInt()!=200)throw std::runtime_error(value["body"]["error"].asString());return value["body"];}}
  throw std::runtime_error("Supervisor response budget exceeded");
}
class RPCServer {
public:
  using Handler=std::function<Json::Value(const Json::Value&)>;
  RPCServer(fs::path path,Handler handler):path_(std::move(path)),handler_(std::move(handler)){
    fs::create_directories(path_.parent_path());auto name=path_.string();sockaddr_un address{};address.sun_family=AF_UNIX;
    if(name.size()>=sizeof(address.sun_path))throw std::invalid_argument("Supervisor socket path too long");std::memcpy(address.sun_path,name.c_str(),name.size()+1);
    struct stat status{};
    if(lstat(path_.c_str(),&status)==0){
      if(!S_ISSOCK(status.st_mode)||status.st_uid!=getuid())throw std::runtime_error("Supervisor socket path is occupied");
      int probe=socket(AF_UNIX,SOCK_STREAM|SOCK_CLOEXEC,0);if(probe<0)throw std::runtime_error("Cannot probe supervisor socket");
      int result=connect(probe,reinterpret_cast<sockaddr*>(&address),sizeof(address));int saved=errno;close(probe);
      if(result==0)throw std::runtime_error("Supervisor socket is already active");if(saved!=ECONNREFUSED&&saved!=ENOENT)throw std::runtime_error("Cannot verify old supervisor socket");fs::remove(path_);
    }
    fd_=socket(AF_UNIX,SOCK_STREAM|SOCK_CLOEXEC,0);if(fd_<0)throw std::runtime_error("Cannot create supervisor socket");
    if(bind(fd_,reinterpret_cast<sockaddr*>(&address),sizeof(address))||chmod(path_.c_str(),0600)||listen(fd_,16)){close(fd_);fd_=-1;throw std::runtime_error("Cannot bind supervisor socket");}
    for(int i=0;i<4;++i)workers_.emplace_back([this](std::stop_token stop){while(!stop.stop_requested()){
      int client=-1;{std::unique_lock<std::mutex> lock(mutex_);condition_.wait_for(lock,std::chrono::milliseconds(100),[&]{return !queue_.empty()||closing_;});if(closing_&&queue_.empty())return;if(queue_.empty())continue;client=queue_.front();queue_.pop_front();}
      serve(client);close(client);
    }});
    acceptor_=std::jthread([this](std::stop_token stop){while(!stop.stop_requested()){
      pollfd state{fd_,POLLIN,0};if(poll(&state,1,100)<=0)continue;int client=accept4(fd_,nullptr,nullptr,SOCK_CLOEXEC);if(client<0)continue;
      std::lock_guard<std::mutex> lock(mutex_);if(queue_.size()>=16)close(client);else{queue_.push_back(client);condition_.notify_one();}
    }});
  }
  ~RPCServer(){acceptor_.request_stop();if(acceptor_.joinable())acceptor_.join();if(fd_>=0)close(fd_);{std::lock_guard<std::mutex> lock(mutex_);closing_=true;}condition_.notify_all();for(auto& thread:workers_)if(thread.joinable())thread.join();for(int fd:queue_)close(fd);std::error_code error;fs::remove(path_,error);}
  RPCServer(const RPCServer&)=delete;RPCServer& operator=(const RPCServer&)=delete;
private:
  fs::path path_;Handler handler_;int fd_=-1;std::jthread acceptor_;std::vector<std::jthread> workers_;std::deque<int> queue_;std::mutex mutex_;std::condition_variable condition_;bool closing_=false;
  void serve(int client){
    timeval timeout{0,500000};setsockopt(client,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));setsockopt(client,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout));
    std::string request;std::array<char,4096> buffer;Json::Value reply;
    try{
      while(request.find('\n')==std::string::npos&&request.size()<=16384){auto n=recv(client,buffer.data(),buffer.size(),0);if(n<=0)return;request.append(buffer.data(),static_cast<size_t>(n));}
      auto end=request.find('\n');if(end==std::string::npos||end>16384)throw std::invalid_argument("Invalid RPC request size");
      auto body=json_parse(request.substr(0,end));if(!body.isObject())throw std::invalid_argument("Expected RPC object");reply["body"]=handler_(body);reply["status"]=200;
    }catch(const std::invalid_argument& e){reply["status"]=400;reply["body"]["error"]=e.what();}
    catch(const std::runtime_error& e){reply["status"]=409;reply["body"]["error"]=e.what();}
    catch(const std::exception&){reply["status"]=503;reply["body"]["error"]="Supervisor unavailable";}
    auto response=json_text(reply)+"\n";size_t offset=0;while(offset<response.size()){auto n=send(client,response.data()+offset,response.size()-offset,MSG_NOSIGNAL);if(n<=0)break;offset+=static_cast<size_t>(n);}
  }
};
}
