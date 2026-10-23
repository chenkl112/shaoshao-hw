#include "shaoshao_runtime/manager.hpp"
#include <iostream>

namespace {volatile std::sig_atomic_t stopped=0;void stop(int){stopped=1;}}
int main(int argc,char** argv){
  using namespace hw_runtime;
  try{
    fs::path root=runtime_path(),maps;std::string role;
    for(int i=1;i<argc;++i){std::string name=argv[i];if(i+1>=argc)throw std::invalid_argument("Missing stack argument");std::string value=argv[++i];if(name=="--runtime")root=value;else if(name=="--role")role=value;else if(name=="--maps-dir")maps=value;else throw std::invalid_argument("Unknown stack argument");}
    if(maps.empty()||(role!="mapping"&&role!="navigation"))throw std::invalid_argument("Provide stack --role and --maps-dir");
    std::signal(SIGINT,stop);std::signal(SIGTERM,stop);Config config(robot_configuration(root));std::vector<Process> children;auto logfile=root/(role+".log");
    auto launch=[&](const std::string& package,const std::string& binary,const std::string& node,const std::vector<std::string>& parameters){
      std::vector<std::string> command{executable(root,package,binary).string(),"--ros-args","-r","__node:="+node};command.insert(command.end(),parameters.begin(),parameters.end());children.push_back(Process::spawn(command,logfile,false));
    };
    auto frame=[&](const std::string& name,const std::string& parent,const std::string& child,const Point& point){
      std::vector<std::string> command{executable(root,"tf2_ros","static_transform_publisher").string(),"--frame-id",parent,"--child-frame-id",child,"--x",std::to_string(point.x()),"--y",std::to_string(point.y()),"--z",std::to_string(point.z()),"--yaw","0","--pitch","0","--roll","0","--ros-args","-r","__node:="+name};children.push_back(Process::spawn(command,logfile,false));
    };
    auto common=std::vector<std::string>{"-p","config_path:="+robot_configuration(root).string(),"-p","maps_dir:="+maps.string(),"-p","runtime_root:="+root.string()};
    if(role=="mapping"){
      launch("shaoshao_hw_runtime","index_recorder","hw_index_recorder",common);
      launch("fast_lio","fastlio_mapping","laserMapping",{"--params-file",(root/"mapping_runtime/fast_lio.yaml").string(),"-r","/Odometry:=/state_estimation"});
      launch("terrain_analysis_ext","terrainAnalysisExt","terrainAnalysisExt",{"--params-file",(root/"mapping_runtime/terrain.yaml").string(),"-r","/registered_scan:=/cloud_registered"});
      frame("mapping_map_odom","map","odom",Point::Zero());
      launch("fast_lio","static_map_projector","static_map_projector",{"-p","resolution:="+std::to_string(config.resolution),"-p","map_path:="+(maps/"site.pcd").string()});
    }else{
      auto params=(root/"navigation_runtime/nav2.yaml").string();auto frames=YAML::LoadFile((root/"navigation_runtime/frames.yaml").string());
      launch("fast_lio","fastlio_mapping","navigation_lio",{"--params-file",(root/"navigation_runtime/fast_lio.yaml").string(),"-r","/Odometry:=/navcar/state_estimation"});
      launch("pointcloud_to_laserscan","pointcloud_to_laserscan_node","navigation_scan",{"--params-file",params,"-r","cloud_in:=/cloud_registered_body","-r","scan:=/navcar/scan"});
      launch("hw_registration","registration_server","hw_registration",{"-p","map_path:="+(maps/"site.pcd").string(),"-p","voxel_m:="+std::to_string(config.registration.voxel),"-p","threads:=2"});
      launch("shaoshao_hw_runtime","localizer","hw_localizer",common);
      launch("hw_omni_controller","obstacle_predictor","hw_obstacle_predictor",{"--params-file",params});
      launch("hw_omni_controller","command_relay","hw_command_relay",{"--params-file",params});
      for(auto t:frames["static_transforms"])frame("navigation_"+t["child"].as<std::string>(),"navcar_imu",t["child"].as<std::string>(),yaml_point(t["xyz"]));
      for(const auto& [package,node]:std::vector<std::pair<std::string,std::string>>{{"nav2_map_server","map_server"},{"nav2_planner","planner_server"},{"nav2_controller","controller_server"},{"nav2_bt_navigator","bt_navigator"}})
        launch(package,node,node,{"--params-file",params,"-r","cmd_vel:=/navcar/cmd_vel_auto"});
      launch("nav2_lifecycle_manager","lifecycle_manager","navigation_lifecycle",{"-p","autostart:=false","-p","use_sim_time:=false","-p","node_names:=[map_server,planner_server,controller_server,bt_navigator]"});
    }
    bool failed=false;
    while(!stopped){for(auto& process:children)if(!process.running()){failed=true;stopped=1;break;}if(!stopped)std::this_thread::sleep_for(std::chrono::milliseconds(50));}
    for(auto it=children.rbegin();it!=children.rend();++it)it->stop(1);
    return failed?1:0;
  }catch(const std::exception& e){std::cerr<<"Native stack failed: "<<e.what()<<'\n';return 1;}
}
