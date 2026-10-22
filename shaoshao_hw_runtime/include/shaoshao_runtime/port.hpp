#pragma once
#include "common.hpp"
#include <cerrno>
#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>

namespace hw_runtime {
class Port {
public:
  explicit Port(const std::string& path){
    fd_=open(path.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK|O_CLOEXEC);if(fd_<0)throw std::runtime_error("Cannot open commissioned serial port");
    try{
      if(flock(fd_,LOCK_EX|LOCK_NB)||ioctl(fd_,TIOCEXCL))throw std::runtime_error("Serial port is already in use");termios options{};if(tcgetattr(fd_,&options))throw std::runtime_error("Cannot read serial settings");cfmakeraw(&options);options.c_cflag|=CLOCAL|CREAD;options.c_cflag&=~CRTSCTS;options.c_cc[VMIN]=0;options.c_cc[VTIME]=0;
      if(cfsetispeed(&options,B2000000)||cfsetospeed(&options,B2000000)||tcsetattr(fd_,TCSANOW,&options))throw std::runtime_error("Cannot configure 2000000-baud UART");tcflush(fd_,TCIFLUSH);
    }catch(...){close(fd_);fd_=-1;throw;}
  }
  ~Port(){if(fd_>=0)close(fd_);}Port(const Port&)=delete;Port& operator=(const Port&)=delete;
  std::string read(){
    pollfd state{fd_,POLLIN,0};if(poll(&state,1,0)>0&&(state.revents&(POLLERR|POLLHUP|POLLNVAL)))throw std::runtime_error("Serial port disconnected");std::array<char,4096> bytes;auto n=::read(fd_,bytes.data(),bytes.size());if(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK||errno==EINTR))return {};if(n<0)throw std::runtime_error("Serial read failed");return std::string(bytes.data(),n>0?static_cast<size_t>(n):0);
  }
  void write(const std::string& bytes){size_t offset=0;double until=steady()+.02;while(offset<bytes.size()){
    auto n=::write(fd_,bytes.data()+offset,bytes.size()-offset);if(n>0){offset+=static_cast<size_t>(n);continue;}
    if(n<0&&errno!=EAGAIN&&errno!=EWOULDBLOCK&&errno!=EINTR)throw std::runtime_error("Serial write failed");if(steady()>=until)throw std::runtime_error("Serial partial write timeout");pollfd writable{fd_,POLLOUT,0};poll(&writable,1,1);
  }}
private:int fd_=-1;
};
}
