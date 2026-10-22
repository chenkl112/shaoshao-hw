#pragma once
#include "common.hpp"
#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>

namespace hw_runtime {
inline bool equal_token(const std::string& a,const std::string& b){size_t difference=a.size()^b.size();for(size_t i=0;i<std::max(a.size(),b.size());++i)difference|=(i<a.size()?static_cast<unsigned char>(a[i]):0)^(i<b.size()?static_cast<unsigned char>(b[i]):0);return difference==0;}
inline Json::Value local_http(int port,const std::string& token,const std::string& path,const std::optional<Json::Value>& body=std::nullopt){
  struct Socket{int fd=-1;~Socket(){if(fd>=0)close(fd);}} socket;
  socket.fd=::socket(AF_INET,SOCK_STREAM|SOCK_CLOEXEC|SOCK_NONBLOCK,0);if(socket.fd<0)throw std::runtime_error("Cannot create controller connection");
  sockaddr_in address{};address.sin_family=AF_INET;address.sin_port=htons(static_cast<uint16_t>(port));inet_pton(AF_INET,"127.0.0.1",&address.sin_addr);
  if(connect(socket.fd,reinterpret_cast<sockaddr*>(&address),sizeof(address))&&errno!=EINPROGRESS)throw std::runtime_error("Controller unavailable");pollfd ready{socket.fd,POLLOUT,0};if(poll(&ready,1,300)<=0)throw std::runtime_error("Controller connection timeout");
  int error=0;socklen_t length=sizeof(error);if(getsockopt(socket.fd,SOL_SOCKET,SO_ERROR,&error,&length)||error)throw std::runtime_error("Controller connection refused");
  fcntl(socket.fd,F_SETFL,fcntl(socket.fd,F_GETFL)&~O_NONBLOCK);timeval timeout{0,300000};setsockopt(socket.fd,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));setsockopt(socket.fd,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout));
  auto payload=body?json_text(*body):"";std::string request=(body?"POST ":"GET ")+path+" HTTP/1.1\r\nHost: 127.0.0.1\r\nAuthorization: Bearer "+token+"\r\nConnection: close\r\n";
  if(body)request+="Content-Type: application/json\r\nContent-Length: "+std::to_string(payload.size())+"\r\n";request+="\r\n"+payload;
  size_t offset=0;while(offset<request.size()){auto n=send(socket.fd,request.data()+offset,request.size()-offset,MSG_NOSIGNAL);if(n<=0)throw std::runtime_error("Controller send failed");offset+=static_cast<size_t>(n);}
  std::string response;std::array<char,8192> buffer;size_t expected=0;double until=steady()+1;
  while(response.size()<1024*1024&&steady()<until){auto n=recv(socket.fd,buffer.data(),buffer.size(),0);if(n==0)break;if(n<0)throw std::runtime_error("Controller receive timeout");response.append(buffer.data(),static_cast<size_t>(n));auto end=response.find("\r\n\r\n");if(end!=std::string::npos&&!expected){auto headers=response.substr(0,end);std::transform(headers.begin(),headers.end(),headers.begin(),[](unsigned char c){return static_cast<char>(std::tolower(c));});auto p=headers.find("content-length:");if(p!=std::string::npos)expected=end+4+std::stoull(headers.substr(p+15));}if(expected&&response.size()>=expected)break;}
  auto end=response.find("\r\n\r\n");if(end==std::string::npos)throw std::runtime_error("Invalid controller HTTP response");std::istringstream status(response.substr(0,response.find("\r\n")));std::string version;int code=0;status>>version>>code;auto value=json_parse(response.substr(end+4));if(code<200||code>=300)throw std::runtime_error(value.get("error","Controller rejected request").asString());return value;
}
}
