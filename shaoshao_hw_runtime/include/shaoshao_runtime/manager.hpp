#pragma once
#include "configuration.hpp"
#include "index.hpp"
#include "process.hpp"
#include "maps.hpp"
#include <mutex>

namespace hw_runtime {
class Manager {
public:
  fs::path runtime,source,maps,maps_base;Config config;bool saved=false;
  std::function<void(const std::string&)> trigger;
  Manager(fs::path root,fs::path initial,bool restore_existing=true):runtime(fs::absolute(root)),source(runtime/"shaoshao"),maps(fs::absolute(initial)),maps_base(maps.parent_path()),config(source/"web_panel/robot.yaml"){
    fs::create_directories(maps);fs::create_directories(runtime);if(restore_existing){restore("mapping",mapping_);restore("navigation",navigation_);}
  }
  ~Manager(){if(mapping_)mapping_->detach();} // A failed save must never stop mapping in a destructor.
  bool mapping_running(){return mapping_&&mapping_->running();}bool navigation_running(){return navigation_&&navigation_->running();}
  std::string mode(){return navigation_running()?"navigation":mapping_running()?"mapping":"idle";}
  void transition(const std::string& target){
    if(target=="idle"){stop_navigation();stop_mapping();}
    else if(target=="mapping"){stop_navigation();start_mapping();}
    else if(target=="navigation"){stop_mapping();start_navigation();}
    else throw std::invalid_argument("Unknown runtime mode");
  }
  void stop_navigation(){if(navigation_){navigation_->stop(8);navigation_.reset();atomic_write(runtime/"navigation.pid","0\n");}}
  void stop_mapping(){
    if(!mapping_running())return;if(!trigger)throw std::runtime_error("Map save services unavailable");
    auto file=maps/"site.pcd";auto before=fs::exists(file)?std::optional(fs::last_write_time(file)):std::nullopt;
    trigger("/map_save");if(!fs::is_regular_file(file)||!fs::file_size(file)||(before&&fs::last_write_time(file)==*before))throw std::runtime_error("PCD 未更新，建图保持运行");
    if(!saved_map_available())throw std::runtime_error("二维地图尚未生成，建图保持运行");
    trigger("/hw/finalize_index");RetrievalIndex verify(maps/"keyframes",file);
    mapping_->stop(15);mapping_.reset();atomic_write(runtime/"mapping.pid","0\n");saved=true;
  }
  void start_mapping(){
    if(mapping_running())return;
    auto filter=mapping_filter(maps);auto count=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    auto next=maps_base/("mid360_mapping_"+std::to_string(count));fs::create_directories(next);write_json(next/"mapping_filter.json",filter);
    write_yaml(runtime/"mapping_runtime/fast_lio.yaml",mapping_configuration(source,next,config));
    Json::Value terrain=json_parse(R"JSON({"terrainAnalysisExt":{"ros__parameters":{"map_frame":"odom","scanVoxelSize":0.1,"decayTime":0.5,"noDecayDis":0.0,"clearingDis":30.0,"useSorting":true,"quantileZ":0.25,"vehicleHeight":1.5,"voxelPointUpdateThre":100,"voxelTimeUpdateThre":2.0,"lowerBoundZ":-3.0,"upperBoundZ":1.0,"disRatioZ":0.1,"checkTerrainConn":false,"localTerrainMapRadius":0.0,"vehicleLength":0.7,"vehicleWidth":0.7}}})JSON");
    write_yaml(runtime/"mapping_runtime/terrain.yaml",yaml_from_json(terrain));
    auto process=Process::spawn({executable(runtime,"shaoshao_hw_runtime","hw_stack").string(),"--runtime",runtime.string(),"--role","mapping","--maps-dir",next.string()},runtime/"mapping.log");
    wait_started(process,1);mapping_=std::move(process);maps=next;saved=false;persist("mapping",*mapping_);atomic_write(runtime/"current_mapping_dir",maps.string()+"\n");
  }
  Json::Value start_navigation(){
    if(navigation_running())return read_frames();RetrievalIndex verify(maps/"keyframes",maps/"site.pcd");
    auto frames=navigation_configuration(runtime,maps,config);
    auto process=Process::spawn({executable(runtime,"shaoshao_hw_runtime","hw_stack").string(),"--runtime",runtime.string(),"--role","navigation","--maps-dir",maps.string()},runtime/"navigation.log");
    wait_started(process,2);navigation_=std::move(process);persist("navigation",*navigation_);return frames;
  }
  Json::Value read_frames() const{return yaml_to_json(YAML::LoadFile((runtime/"navigation_runtime/frames.yaml").string()));}
  pid_t ensure_driver(){
    for(const auto& entry:fs::directory_iterator("/proc")){
      auto text=entry.path().filename().string();if(text.empty()||!std::all_of(text.begin(),text.end(),[](char c){return std::isdigit(static_cast<unsigned char>(c));}))continue;
      pid_t pid=static_cast<pid_t>(std::stol(text));auto command=process_command(pid);auto end=command.find('\0');if(end==std::string::npos)continue;
      if(fs::path(command.substr(0,end)).filename()!="livox_ros_driver2_node")continue;
      if(command.find("xfer_format:=1")!=std::string::npos&&command.find("MID360_config.json")!=std::string::npos)return pid;
      throw std::runtime_error("A different Livox driver is running; stop it before changing format");
    }
    auto driver=Process::spawn({executable(runtime,"livox_ros_driver2","livox_ros_driver2_node").string(),"--ros-args","-p","xfer_format:=1","-p","multi_topic:=0","-p","data_src:=0","-p","publish_freq:=10.0","-p","output_data_type:=0","-p","frame_id:=livox_frame","-p","user_config_path:="+(runtime/"config/MID360_config.json").string()},runtime/"lidar.log");
    wait_started(driver,1);auto pid=driver.detach();atomic_write(runtime/"lidar.pid",std::to_string(pid)+"\n");return pid;
  }
private:
  std::optional<Process> mapping_,navigation_;
  bool saved_map_available() const{return saved_map(maps).info["available"].asBool();}
  void persist(const std::string& kind,const Process& process){Json::Value value;value["pid"]=process.pid();value["start_ticks"]=Json::UInt64(process.start());value["maps_dir"]=maps.string();write_json(runtime/(kind+"-session.json"),value);atomic_write(runtime/(kind+".pid"),std::to_string(process.pid())+"\n");}
  void restore(const std::string& kind,std::optional<Process>& output){auto file=runtime/(kind+"-session.json");if(!fs::exists(file))return;auto session=read_json(file);pid_t pid=session["pid"].asInt();if(process_start(pid)!=session["start_ticks"].asUInt64())return;
    auto marker="--role"+std::string(1,'\0')+kind;if(process_command(pid).find(marker)==std::string::npos)throw std::runtime_error("Stored runtime process does not match its role");
    output=Process::attach(pid,session["start_ticks"].asUInt64(),marker);maps=fs::absolute(session["maps_dir"].asString());
  }
  static void wait_started(Process& process,double delay){double until=steady()+delay;while(steady()<until){if(!process.running())throw std::runtime_error("Native stack exited during startup; inspect runtime logs");std::this_thread::sleep_for(std::chrono::milliseconds(50));}}
};
}
