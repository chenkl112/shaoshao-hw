#pragma once
#include "common.hpp"
#include <atomic>
#include <cerrno>
#include <csignal>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/wait.h>
#include <thread>

namespace hw_runtime {
inline std::string process_command(pid_t pid){try{return read_text(fs::path("/proc")/std::to_string(pid)/"cmdline",65536);}catch(const std::exception&){return {};}}
inline uint64_t process_start(pid_t pid){try{auto s=read_text(fs::path("/proc")/std::to_string(pid)/"stat",8192);auto end=s.rfind(')');if(end==std::string::npos)return 0;std::istringstream in(s.substr(end+2));std::string value;for(int field=3;field<=22;++field){if(!(in>>value))return 0;if(field==22)return std::stoull(value);}}catch(const std::exception&){}return 0;}
inline bool group_alive(pid_t group){
  std::error_code error;for(const auto& entry:fs::directory_iterator("/proc",error)){
    auto name=entry.path().filename().string();if(name.empty()||!std::all_of(name.begin(),name.end(),[](char c){return c>='0'&&c<='9';}))continue;
    try{auto text=read_text(entry.path()/"stat",8192);auto end=text.rfind(')');std::istringstream in(text.substr(end+2));char state;pid_t parent,pgid;in>>state>>parent>>pgid;if(state!='Z'&&pgid==group)return true;}catch(const std::exception&){}
  }return false;
}
class FileLock {
public:
  explicit FileLock(const fs::path& path){fs::create_directories(path.parent_path());fd_=open(path.c_str(),O_RDWR|O_CREAT|O_CLOEXEC,0600);if(fd_<0||flock(fd_,LOCK_EX|LOCK_NB)){if(fd_>=0)close(fd_);throw std::runtime_error("Runtime already owned: "+path.string());}}
  ~FileLock(){if(fd_>=0)close(fd_);}FileLock(const FileLock&)=delete;FileLock& operator=(const FileLock&)=delete;
private:int fd_=-1;
};
class Process {
public:
  Process()=default;Process(const Process&)=delete;Process& operator=(const Process&)=delete;
  Process(Process&& other) noexcept {take(other);}Process& operator=(Process&& other) noexcept {if(this!=&other){try{stop();}catch(...){}take(other);}return *this;}
  ~Process(){try{stop();}catch(...) {}}
  static Process spawn(const std::vector<std::string>& command,const fs::path& logfile,bool grouped=true){
    if(command.empty())throw std::invalid_argument("Empty process command");fs::create_directories(logfile.parent_path());int log=open(logfile.c_str(),O_WRONLY|O_CREAT|O_APPEND|O_CLOEXEC,0600);if(log<0)throw std::runtime_error("Cannot open process log");
    std::vector<char*> argv;for(const auto& arg:command)argv.push_back(const_cast<char*>(arg.c_str()));argv.push_back(nullptr);
    pid_t pid=fork();if(pid<0){close(log);throw std::runtime_error("fork failed");}
    if(pid==0){if(grouped&&setsid()<0)_exit(126);if(dup2(log,STDOUT_FILENO)<0||dup2(log,STDERR_FILENO)<0)_exit(126);close(log);int null=open("/dev/null",O_RDONLY);if(null>=0){dup2(null,STDIN_FILENO);close(null);}execvp(argv[0],argv.data());_exit(127);}
    close(log);Process out;out.pid_=pid;out.grouped_=grouped;out.owned_=true;
    for(int i=0;i<20;++i){out.start_=process_start(pid);if(out.start_)break;std::this_thread::sleep_for(std::chrono::milliseconds(1));}
    return out;
  }
  static Process attach(pid_t pid,uint64_t start,const std::string& marker){
    if(pid<=1||!start||process_start(pid)!=start||process_command(pid).find(marker)==std::string::npos||getpgid(pid)!=pid)throw std::runtime_error("Process identity changed; refusing to signal it");
    Process out;out.pid_=pid;out.start_=start;out.grouped_=true;out.owned_=true;return out;
  }
  pid_t pid() const{return pid_;}uint64_t start() const{return start_;}
  bool running(){
    if(pid_<=1)return false;int status=0;pid_t result=waitpid(pid_,&status,WNOHANG);
    if(result==pid_){exit_status_=status;return false;}
    if(process_start(pid_)!=start_)return false;
    return kill(pid_,0)==0;
  }
  bool healthy_group(){return running()||(grouped_&&owned_&&group_alive(pid_));}
  void stop(double grace=10){
    if(!owned_||pid_<=1)return;bool parent=running();
    if(parent)kill(pid_,SIGINT);
    double until=steady()+grace;while(steady()<until&&healthy_group())std::this_thread::sleep_for(std::chrono::milliseconds(20));
    if(healthy_group())signal_group(SIGTERM);
    until=steady()+2;while(steady()<until&&healthy_group())std::this_thread::sleep_for(std::chrono::milliseconds(20));
    if(healthy_group())signal_group(SIGKILL);
    until=steady()+2;while(steady()<until&&healthy_group())std::this_thread::sleep_for(std::chrono::milliseconds(10));
    bool alive=healthy_group();owned_=false;if(alive)throw std::runtime_error("Owned process group failed to exit");
  }
  pid_t detach(){owned_=false;return pid_;}
private:
  pid_t pid_=0;uint64_t start_=0;bool grouped_=true,owned_=false;std::optional<int> exit_status_;
  void signal_group(int signal){if(grouped_){if(group_alive(pid_))kill(-pid_,signal);}else if(process_start(pid_)==start_)kill(pid_,signal);}
  void take(Process& other){pid_=other.pid_;start_=other.start_;grouped_=other.grouped_;owned_=other.owned_;exit_status_=other.exit_status_;other.owned_=false;other.pid_=0;}
};
inline fs::path executable(const fs::path& runtime,const std::string& package,const std::string& name){
  // colcon installs package executables below <prefix>/lib/<package>/<name>.
  // Keep the runtime prefix first so native services work even when the
  // caller did not source mapping_install/setup.bash.
  std::vector<fs::path> prefixes{runtime/"mapping_install"};
  if(const char* environment=std::getenv("AMENT_PREFIX_PATH")){std::istringstream list(environment);std::string prefix;while(std::getline(list,prefix,':'))prefixes.emplace_back(prefix);}
  prefixes.emplace_back("/opt/ros/jazzy");for(const auto& prefix:prefixes){auto binary=prefix/"lib"/package/name;if(fs::is_regular_file(binary)&&access(binary.c_str(),X_OK)==0)return binary;}
  throw std::runtime_error("Required native executable not found: "+package+"/"+name);
}
}
