#pragma once
#include "common.hpp"

namespace hw_runtime {
inline YAML::Node yaml_from_json(const Json::Value& value){return YAML::Load(json_text(value));}
inline Json::Value yaml_to_json(const YAML::Node& node){
  if(!node||node.IsNull())return Json::Value();if(node.IsSequence()){Json::Value out(Json::arrayValue);for(auto item:node)out.append(yaml_to_json(item));return out;}
  if(node.IsMap()){Json::Value out(Json::objectValue);for(auto item:node)out[item.first.as<std::string>()]=yaml_to_json(item.second);return out;}
  auto text=node.Scalar();if(text=="true"||text=="True")return Json::Value(true);if(text=="false"||text=="False")return Json::Value(false);
  try{size_t length=0;double value=std::stod(text,&length);if(length==text.size()&&std::isfinite(value)){if(text.find_first_of(".eE")==std::string::npos)return Json::Value(Json::Int64(std::stoll(text)));return Json::Value(value);}}catch(const std::exception&){}
  return Json::Value(text);
}
inline Json::Value mapping_filter(const fs::path& maps){
  Json::Value value;if(fs::is_regular_file(maps/"mapping_filter.json"))value=read_json(maps/"mapping_filter.json");else{value["enabled"]=true;value["height_m"]=1.5;value["dynamic_enabled"]=true;}
  if(!value["enabled"].isBool()||!value["dynamic_enabled"].isBool()||finite_number(value,"height_m")<=0||value["height_m"].asDouble()>50)throw std::invalid_argument("Invalid mapping filter");return value;
}
inline YAML::Node mapping_configuration(const fs::path& source,const fs::path& maps,const Config& config){
  auto root=YAML::LoadFile((source/"sentry_bringup/params/fast_lio_mapping_param.yaml").string());auto parameters=root["/**"]["ros__parameters"];
  auto filter=mapping_filter(maps);if(!filter["dynamic_enabled"].asBool())throw std::invalid_argument("Current mapping runner requires static dynamic filtering");
  parameters["map_file_path"]=(maps/"site.pcd").string();parameters["filter_size_surf"]=.15;parameters["filter_size_map"]=.15;
  parameters["preprocess"]["lidar_type"]=1;parameters["preprocess"]["blind"]=.2;
  parameters["common"]["odom_frame_id"]="odom";parameters["common"]["sensor_frame_id"]="imu_link";parameters["common"]["send_odom_base_tf"]=false;
  parameters["mapping"]["extrinsic_est_en"]=false;
  for(int i=0;i<3;++i)parameters["mapping"]["extrinsic_T"][i]=config.lidar_offset[i];
  for(int i=0;i<9;++i)parameters["mapping"]["extrinsic_R"][i]=(i%4==0)?1.:0.;
  parameters["map_filter"]["enabled"]=filter["enabled"].asBool();parameters["map_filter"]["max_height"]=filter["height_m"].asDouble();parameters["map_filter"]["dynamic_enabled"]=true;parameters["map_filter"]["static_resolution"]=config.resolution;
  return YAML::Clone(root);
}
inline Json::Value navigation_configuration(const fs::path& runtime,const fs::path& maps,Config config){
  auto source=runtime/"shaoshao",output=runtime/"navigation_runtime";fs::create_directories(output);
  auto metadata=YAML::LoadFile((maps/"site.yaml").string());double resolution=metadata["resolution"].as<double>();
  if(!std::isfinite(resolution)||std::abs(resolution-config.resolution)>1e-8)throw std::invalid_argument("地图分辨率必须与配置一致；不能仅修改地图 YAML");
  auto lio=YAML::LoadFile((runtime/"mapping_runtime/fast_lio.yaml").string());auto p=lio["/**"]["ros__parameters"];
  config.lidar_offset=yaml_point(p["mapping"]["extrinsic_T"]);auto rotation=p["mapping"]["extrinsic_R"];
  if(!rotation.IsSequence()||rotation.size()!=9)throw std::invalid_argument("Invalid lidar extrinsic rotation");
  for(int i=0;i<9;++i)if(std::abs(rotation[i].as<double>()-((i%4==0)?1.:0.))>1e-8)throw std::invalid_argument("Measured center frame requires aligned lidar/IMU axes");
  auto xy=config.yaml["yaw"]["lidar_offset_xy_m"];config.center_offset<<config.lidar_offset.x()-xy[0].as<double>(),config.lidar_offset.y()-xy[1].as<double>(),0;
  Json::Value frames;frames["reference"]="yaw_axis";frames["resolution"]=resolution;frames["min_range"]=.2;frames["position_std_limit"]=.01;
  frames["base_frame"]="navcar_center";frames["center_offset"]=point_json(config.center_offset);frames["maps_dir"]=fs::absolute(maps).string();frames["robot_radius_m"]=config.radius;frames["footprint_padding_m"]=.02;
  for(const auto& [name,offset]:std::vector<std::pair<std::string,Point>>{{"navcar_center",config.center_offset},{"navcar_lidar",config.lidar_offset}}){Json::Value t;t["child"]=name;t["xyz"]=point_json(offset);frames["static_transforms"].append(t);}
  write_yaml(output/"frames.yaml",yaml_from_json(frames));
  p["preprocess"]["blind"]=.2;p["common"]["odom_frame_id"]="navcar_odom";p["common"]["sensor_frame_id"]="navcar_imu";p["common"]["send_odom_base_tf"]=false;
  p["pcd_save"]["pcd_save_en"]=false;p["map_filter"]["dynamic_enabled"]=false;p["locate_in_prior_map"]=false;p["publish"]["scan_bodyframe_pub_en"]=true;p["publish"]["map_en"]=false;p["map_file_path"]=(output/"navigation-unused.pcd").string();
  write_yaml(output/"fast_lio.yaml",YAML::Clone(lio));
  Json::Value parameters=json_parse(R"JSON({
    "navigation_scan":{"ros__parameters":{"use_sim_time":false,"target_frame":"navcar_lidar","transform_tolerance":0.1,"min_height":-0.227,"max_height":1.2,"angle_min":-3.141592653589793,"angle_max":3.141592653589793,"angle_increment":0.008726646259971648,"scan_time":0.1,"range_min":0.2,"range_max":15.0,"use_inf":true}},
    "map_server":{"ros__parameters":{"use_sim_time":false,"frame_id":"map"}},
    "planner_server":{"ros__parameters":{"use_sim_time":false,"expected_planner_frequency":1.0,"planner_plugins":["GridBased"],"GridBased":{"plugin":"nav2_navfn_planner::NavfnPlanner","tolerance":0.02,"use_astar":true,"allow_unknown":false}}},
    "controller_server":{"ros__parameters":{"use_sim_time":false,"controller_frequency":20.0,"odom_topic":"/navcar/odom","odom_duration":0.1,"speed_limit_topic":"/navcar/speed_limit","enable_stamped_cmd_vel":true,"min_x_velocity_threshold":0.001,"min_y_velocity_threshold":0.001,"min_theta_velocity_threshold":0.001,"failure_tolerance":0.3,"progress_checker_plugins":["progress_checker"],"goal_checker_plugins":["goal_checker"],"controller_plugins":["FollowPath"],"progress_checker":{"plugin":"nav2_controller::SimpleProgressChecker","required_movement_radius":0.01,"movement_time_allowance":20.0},"goal_checker":{"plugin":"nav2_controller::StoppedGoalChecker","xy_goal_tolerance":0.03,"yaw_goal_tolerance":6.3,"stateful":true,"trans_stopped_velocity":0.03,"rot_stopped_velocity":100.0}}},
    "bt_navigator":{"ros__parameters":{"use_sim_time":false,"global_frame":"map","robot_base_frame":"navcar_center","odom_topic":"/navcar/odom","bt_loop_duration":20,"default_server_timeout":1000,"wait_for_service_timeout":5000,"navigators":["navigate_to_pose"],"navigate_to_pose":{"plugin":"nav2_bt_navigator::NavigateToPoseNavigator"}}}
  })JSON");
  parameters["navigation_scan"]["ros__parameters"]["min_height"]=.1-config.lidar_height;
  parameters["map_server"]["ros__parameters"]["yaml_filename"]=(maps/"site.yaml").string();
  auto& controller=parameters["controller_server"]["ros__parameters"]["FollowPath"];
  controller["plugin"]="hw_omni_controller::Controller";controller["max_speed_mps"]=config.speed;controller["acceleration_mps2"]=std::min(config.acceleration,config.deceleration);controller["jerk_mps3"]=config.jerk;controller["collision_radius_m"]=config.radius+.02;controller["tau_x_s"]=config.tau_x;controller["tau_y_s"]=config.tau_y;controller["feedback_timeout_s"]=config.feedback_timeout;controller["prediction_timeout_s"]=config.prediction_timeout;controller["obstacle_margin_m"]=config.obstacle_margin;
  Json::Value local=json_parse(R"JSON({"use_sim_time":false,"global_frame":"map","robot_base_frame":"navcar_center","update_frequency":10.0,"publish_frequency":10.0,"rolling_window":true,"width":4,"height":4,"track_unknown_space":true,"always_send_full_costmap":true,"footprint_padding":0.0,"transform_tolerance":0.3,"plugins":["obstacle_layer","inflation_layer"],"obstacle_layer":{"plugin":"nav2_costmap_2d::ObstacleLayer","enabled":true,"observation_sources":"scan","scan":{"topic":"/navcar/scan","data_type":"LaserScan","clearing":true,"marking":true,"max_obstacle_height":2.0,"raytrace_max_range":15.0,"raytrace_min_range":0.2,"obstacle_max_range":10.0,"obstacle_min_range":0.2,"inf_is_valid":false}},"inflation_layer":{"plugin":"nav2_costmap_2d::InflationLayer","cost_scaling_factor":5.0}})JSON");
  local["resolution"]=resolution;local["robot_radius"]=config.radius;local["inflation_layer"]["inflation_radius"]=config.radius+.2;
  Json::Value footprint(Json::arrayValue);double radius=(config.radius+.02)/std::cos(pi/64);
  for(int i=0;i<64;++i){double angle=(i+.5)*2*pi/64;Json::Value point(Json::arrayValue);point.append(radius*std::cos(angle));point.append(radius*std::sin(angle));footprint.append(point);}local["footprint"]=json_text(footprint);
  auto global=local;global["rolling_window"]=false;global["update_frequency"]=1.;global["plugins"]=json_parse("[\"static_layer\",\"obstacle_layer\",\"inflation_layer\"]");global["static_layer"]["plugin"]="nav2_costmap_2d::StaticLayer";global["static_layer"]["map_subscribe_transient_local"]=true;
  parameters["local_costmap"]["local_costmap"]["ros__parameters"]=local;parameters["global_costmap"]["global_costmap"]["ros__parameters"]=global;
  parameters["bt_navigator"]["ros__parameters"]["default_nav_to_pose_bt_xml"]=(source/"web_panel/navcar_translation.xml").string();
  Json::Value obstacles=config.yaml["navigation"]["obstacles"]?yaml_to_json(config.yaml["navigation"]["obstacles"]):Json::Value(Json::objectValue);
  obstacles["map_path"]=(maps/"site.pcd").string();obstacles["lidar_height_m"]=config.lidar_height;parameters["hw_obstacle_predictor"]["ros__parameters"]=obstacles;
  auto& relay=parameters["hw_command_relay"]["ros__parameters"];relay["max_speed_mps"]=config.speed;relay["deceleration_mps2"]=std::min(config.acceleration,config.deceleration);relay["jerk_mps3"]=config.jerk;relay["collision_radius_m"]=config.radius+.02;relay["feedback_timeout_s"]=config.feedback_timeout;
  write_yaml(output/"nav2.yaml",yaml_from_json(parameters));return frames;
}
inline void configure_runtime(const fs::path& source,const fs::path& runtime){
  fs::create_directories(runtime/"config");fs::create_directories(runtime/"mapping_runtime");fs::create_directories(runtime/"navigation_runtime");Config robot(source/"web_panel/robot.yaml");
  auto serial=YAML::LoadFile((source/"web_panel/serial.navcar.example.yaml").string());serial["serial_port"]=robot.yaml["hardware"]["serial_port"].as<std::string>();serial["baud"]=robot.yaml["hardware"]["baud"].as<int>();
  if(!fs::exists(runtime/"config/serial.navcar.yaml"))write_yaml(runtime/"config/serial.navcar.yaml",serial);
  auto lidar=read_json(source/"deployment/config/MID360_config.json");auto host=robot.yaml["network"]["host_ip"].as<std::string>();
  for(const auto& key:{"cmd_data_ip","push_msg_ip","point_data_ip","imu_data_ip"})lidar["MID360"]["host_net_info"][key]=host;
  lidar["lidar_configs"][0]["ip"]=robot.yaml["network"]["lidar_ip"].as<std::string>();if(!fs::exists(runtime/"config/MID360_config.json"))write_json(runtime/"config/MID360_config.json",lidar);
}
}
