#include "shaoshao_runtime/manager.hpp"
#include "shaoshao_runtime/rpc.hpp"
#include <iostream>
#include <sys/stat.h>

namespace hw_runtime {
inline std::string token_file(const fs::path& path){
  if(fs::is_regular_file(path)){auto value=read_text(path,1024);while(!value.empty()&&std::isspace(static_cast<unsigned char>(value.back())))value.pop_back();if(value.empty())throw std::runtime_error("Empty token file");return value;}
  auto token=random_token();atomic_write(path,token+"\n");chmod(path.c_str(),0600);return token;
}
}
int main(int argc,char** argv){
  using namespace hw_runtime;
  try{
    if(argc<2)throw std::invalid_argument("Usage: hw_runtime configure|preview|start-mapping|stop-mapping|status|prepare-navigation|check-index");
    std::string operation=argv[1];fs::path runtime=runtime_path(),source,maps;bool check=false;
    if(operation=="configure"){
      if(argc!=4)throw std::invalid_argument("Usage: hw_runtime configure SOURCE RUNTIME");configure_runtime(fs::absolute(argv[2]),fs::absolute(argv[3]));std::cout<<"Native runtime configuration ready\n";return 0;
    }
    for(int i=2;i<argc;++i){std::string name=argv[i];if(name=="--check"){check=true;continue;}if(i+1>=argc)throw std::invalid_argument("Missing runtime argument");auto value=fs::path(argv[++i]);if(name=="--runtime")runtime=value;else if(name=="--maps-dir")maps=value;else throw std::invalid_argument("Unknown runtime argument");}
    runtime=fs::absolute(runtime);
    if(maps.empty()){
      if(fs::is_regular_file(runtime/"current_mapping_dir")){auto name=read_text(runtime/"current_mapping_dir",4096);while(!name.empty()&&std::isspace(static_cast<unsigned char>(name.back())))name.pop_back();maps=name;}
      else{const char* home=std::getenv("HOME");if(!home)throw std::runtime_error("HOME unavailable");maps=fs::path(home)/"scurm_maps/shaoshao-hw/preview";fs::create_directories(maps);atomic_write(runtime/"current_mapping_dir",fs::absolute(maps).string()+"\n");}
    }
    maps=fs::absolute(maps);
    if(operation=="prepare-navigation"){Config config(runtime/"shaoshao/web_panel/robot.yaml");std::cout<<json_text(navigation_configuration(runtime,maps,config))<<'\n';return 0;}
    if(operation=="check-index"){RetrievalIndex index(maps/"keyframes",maps/"site.pcd");std::cout<<"Map/index hash and format verified\n";return 0;}
    if(operation=="preview"){
      FileLock lock(runtime/"web-start.lock");
      auto state=runtime/"web-session.json";
      if(fs::is_regular_file(state)){auto saved=read_json(state);pid_t pid=saved["pid"].asInt();if(process_start(pid)==saved["start_ticks"].asUInt64()&&process_command(pid).find("hw_supervisor")!=std::string::npos){std::cout<<"网页已运行，保留当前模式。\nWEB http://<vehicle-ip>:8080/?token="<<token_file(runtime/"web.token")<<'\n';return 0;}}
      if(fs::is_regular_file(runtime/"web.pid")){pid_t pid=static_cast<pid_t>(std::stol(read_text(runtime/"web.pid",64)));if(pid>1&&process_start(pid))throw std::runtime_error("An older web service is running; stop it before switching to the native runtime");}
      Manager manager(runtime,maps,false);manager.ensure_driver();
      auto process=Process::spawn({executable(runtime,"shaoshao_hw_runtime","hw_supervisor").string(),"--runtime",runtime.string(),"--maps-dir",maps.string()},runtime/"web.log");
      std::this_thread::sleep_for(std::chrono::seconds(1));if(!process.running())throw std::runtime_error("Native supervisor exited; inspect web.log");
      Json::Value session;session["pid"]=process.pid();session["start_ticks"]=Json::UInt64(process.start());write_json(state,session);atomic_write(runtime/"web.pid",std::to_string(process.pid())+"\n");process.detach();
      std::cout<<"实时点云网页已启动，默认待机。\nWEB http://<vehicle-ip>:8080/?token="<<token_file(runtime/"web.token")<<'\n';return 0;
    }
    Json::Value request;request["operation"]="snapshot";
    if(operation=="status"||check){std::cout<<json_text(rpc_call(runtime/"web-supervisor.sock",request))<<'\n';return 0;}
    if(operation!="start-mapping"&&operation!="stop-mapping")throw std::invalid_argument("Unknown runtime operation");
    request["operation"]="command";request["path"]="/api/mode";request["body"]["mode"]=operation=="start-mapping"?"mapping":"idle";rpc_call(runtime/"web-supervisor.sock",request);
    request=Json::Value();request["operation"]="snapshot";double until=steady()+90;
    while(steady()<until){auto result=rpc_call(runtime/"web-supervisor.sock",request);const auto& state=result["state"];if(!state["switching"].asBool()){
      if(!state["error"].asString().empty())throw std::runtime_error(state["error"].asString());std::cout<<json_text(result)<<'\n';return 0;}
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    throw std::runtime_error("Native mode transition timed out; inspect runtime state");
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
