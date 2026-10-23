#include "shaoshao_runtime/configuration.hpp"
#include "shaoshao_runtime/index.hpp"
#include "shaoshao_runtime/pcd.hpp"
#include "shaoshao_runtime/policy.hpp"
#include "shaoshao_runtime/process.hpp"
#include "shaoshao_runtime/serial.hpp"
#include "shaoshao_runtime/tasks.hpp"
#include "shaoshao_runtime/manager.hpp"
#include <iostream>
#include <random>

using namespace hw_runtime;
void require(bool yes,const char* message){if(!yes)throw std::runtime_error(message);}
template<class F>void rejects(F action,const char* message){bool rejected=false;try{action();}catch(const std::exception&){rejected=true;}require(rejected,message);}
Metrics good(double error=.01){return {true,error,.95,2000,500};}
void observed(Policy& p,double t,double x=0,double clearance=2){p.odometry(pose(x),t);std::vector<Eigen::Vector2d> scan;for(int i=0;i<60;++i)scan.emplace_back(clearance*std::cos(i*pi/30),clearance*std::sin(i*pi/30));p.scan(t,scan);}
Policy acquired(){Policy p;for(double t:{1.,1.1,1.2}){observed(p,t);p.observe(pose(1,2),good(),t,t,true);}return p;}
void localization(){
  Policy p;observed(p,1);require(!p.observe(pose(),good(),1,1,true),"First confirmation must not activate");require(!p.observe(pose(),good(),1,1.1,true)&&p.confirmations==1,"Replayed scan cannot confirm twice");
  p.observe(pose(),good(),1.1,1.1,true);require(!p.correction,"Two confirmations cannot activate");require(p.observe(pose(),good(),1.2,1.2,true),"Three independent confirmations activate");
  p=acquired();auto before=*p.correction;observed(p,1.3,.04);require(p.observe(pose(1,2),good(),1.3,1.3),"Local correction accepted");require(delta(before,*p.correction).first<1e-12,"Robot motion must not alter map correction");
  require(!p.observe(pose(1.5,2),good(),1.4,1.4),"Reject correction jump");require(*p.accepted_at==1.3,"Rejected result must not renew correction age");require(p.status(1.4,.297658,.02,.5)["state"].asString()=="degraded","Rejected correction causes degraded state");
  p=acquired();p.reject("bad",1.3);observed(p,1.3,0,.325);require(!p.status(1.3,.297658,.02,.5)["ready"].asBool(),"Narrow clearance blocks degraded motion");
  p=acquired();observed(p,3.3);require(p.status(3.3,.297658,.02,.5)["state"].asString()=="lost","Correction timeout must stop");
  require(!unique_best({{pose(),good(.01)},{pose(2),good(.011)}},RegistrationSettings{}),"Ambiguous global registration cannot activate");
}
void indices(const fs::path& root){
  std::mt19937 engine(7);std::uniform_real_distribution<double> x(-4,5),y(-3,4),z(-.3,1.2);Cloud points;
  for(int i=0;i<5000;++i)points.emplace_back(x(engine),y(engine),z(engine));auto target=pose(1,2,.2);Cloud mapped;for(const auto& p:points)mapped.push_back(target*p);
  auto maps=root/"maps";fs::create_directories(maps);save_pcd(maps/"site.pcd",mapped);require(read_pcd(maps/"site.pcd").size()==5000,"Native PCD round trip");
  IndexWriter writer(maps/"keyframes");require(writer.add(points,target,1),"Record first frame");require(!writer.add(points,target,2),"Do not duplicate stationary keyframes");writer.finalize(maps/"site.pcd",read_pcd(maps/"site.pcd"));
  RetrievalIndex index(maps/"keyframes",maps/"site.pcd");auto rotation=pose(0,0,pi/3);Cloud query;for(const auto& p:points)query.push_back(rotation.inverse()*p);
  auto candidates=index.candidates(query,Pose::Identity());require(!candidates.empty(),"Retrieve indexed place");auto [distance,angle]=delta(candidates[0],target*rotation);require(distance<.01&&angle<.11,"Recover descriptor yaw and map position");
  auto np=read_npz(maps/"keyframes/index.npz","contexts");require(np.shape==std::vector<size_t>({1,20,60}),"Keep NPY index dimensions");
  std::ofstream(maps/"site.pcd",std::ios::app)<<"changed";rejects([&]{RetrievalIndex wrong(maps/"keyframes",maps/"site.pcd");},"Map/index hash mismatch must reject navigation");
}
void uart(){
  using namespace hw_runtime::serial;
  auto packet=controls(65535,{1,-2,0,17});Parser parser;std::vector<Packet> packets;
  std::string state=frame(0x81,1,packet.substr(8,16));double t=1;
  for(char c:state){auto parsed=parser.feed(std::string(1,c),t);packets.insert(packets.end(),parsed.begin(),parsed.end());t+=.0001;}
  require(packets.size()==1&&packets[0].values[1]==-2,"Parse fragmented UART floats");
  auto corrupt=state;corrupt[10]^=1;require(parser.feed(corrupt+state,t).size()==1,"CRC corruption resynchronizes to the next frame");
  Session session;session.receive(state,1);require(session.sequence&&*session.sequence==2,"Synchronize next command sequence");auto first=session.poll({0,0,0,17},1);require(!first.empty(),"Send first synchronized target");session.receive(frame(0x80,2,std::string(1,'\0')),1.01);require(session.pending.empty()&&*session.sequence==3,"ACK advances sequence");
  require(session.poll({.1,0,0,17},1.02).size()>0,"Subsequent command");session.poll({.1,0,0,17},1.4);require(!session.error.empty(),"Feedback outage blocks sending");
  Settings legacy;legacy.v2=false;legacy.transport="navcar_uart1_v1";legacy.forward=legacy.left=10;legacy.max_translation=30;
  Controller c(legacy);c.receive(frame(0x81,1,controls(1,{0,0,0,17}).substr(8,16)),10);auto lease=c.acquire(10);
  Json::Value command;command["lease"]=lease;command["seq"]=1;command["forward"]=.3;command["left"]=.4;c.navigation_command(command,10);
  auto native=c.desired(10);require(std::abs(native[0]-3)<1e-6&&std::abs(native[1]+4)<1e-6,"Apply UART multiplier exactly once");
  Json::Value manual;manual["action"]="stop";manual["strength"]=.1;manual["client"]="operator";manual["seq"]=1;c.manual(manual,10.01);
  command["seq"]=2;rejects([&]{c.navigation_command(command,10.02);},"Manual takeover revokes previous navigation lease");require(c.desired(10.02)[0]==0,"Stop command takes effect");
}
void configurations(const fs::path& root,const fs::path& source){
  auto runtime=root/"runtime";fs::create_directories(runtime);fs::create_directory_symlink(source,runtime/"shaoshao");configure_runtime(source,runtime);Config config(source/"web_panel/robot.yaml");
  auto maps=root/"new-map";fs::create_directories(maps);write_yaml(runtime/"mapping_runtime/fast_lio.yaml",mapping_configuration(source,maps,config));
  Json::Value filter;filter["enabled"]=true;filter["dynamic_enabled"]=true;filter["height_m"]=2.0;write_json(maps/"mapping_filter.json",filter);
  write_yaml(runtime/"mapping_runtime/fast_lio.yaml",mapping_configuration(source,maps,config));
  auto lio=YAML::LoadFile((runtime/"mapping_runtime/fast_lio.yaml").string());
  require(lio["/**"]["ros__parameters"]["map_filter"]["max_height"].Scalar()=="2.0","Integral-valued ROS doubles keep their YAML type");
  require(lio["/**"]["ros__parameters"]["mapping"]["extrinsic_R"][0].Scalar()=="1.0", "ROS rotation parameters must retain double_array type");
  config.yaml["navigation"]["obstacles"]["range_m"]=8;
  atomic_write(maps/"site.yaml","image: site.png\nresolution: 0.025\n");auto frames=navigation_configuration(runtime,maps,config);
  require(std::abs(frames["center_offset"][0].asDouble()-.00318)<1e-8,"Preserve measured reference offset");auto nav=YAML::LoadFile((runtime/"navigation_runtime/nav2.yaml").string());
  require(nav["controller_server"]["ros__parameters"]["FollowPath"]["plugin"].as<std::string>()=="hw_omni_controller::Controller","Native configuration keeps FDDP");require(nav["local_costmap"]["local_costmap"]["ros__parameters"]["global_frame"].as<std::string>()=="map","Native configuration keeps map-frame predictions");
  require(read_text(runtime/"navigation_runtime/nav2.yaml").find("&1")==std::string::npos,"No ROS YAML anchors");
  require(nav["hw_obstacle_predictor"]["ros__parameters"]["range_m"].Scalar()=="8.0","Integer-valued obstacle settings retain ROS double type");
  atomic_write(maps/"site.yaml","image: site.png\nresolution: 0.05\n");rejects([&]{navigation_configuration(runtime,maps,config);},"Old map resolution cannot silently change");
}
void processes(const fs::path& root){
  auto child=Process::spawn({"/bin/sleep","30"},root/"sleep.log");require(child.running(),"Native process starts");child.stop(.2);require(!child.running(),"Native process stops and reaps");
  rejects([&]{auto foreign=Process::attach(getpid(),0,"not-this-process");},"Never signal a foreign/reused process");
  FileLock owner(root/"runtime.lock");rejects([&]{FileLock duplicate(root/"runtime.lock");},"Exclusive runtime ownership");
}
void tasks(const fs::path& root){
  auto maps=root/"task-map";std::vector<int8_t> grid(10000,0);grid[5050]=100;
  require(save_grid(maps,100,100,.025,Point::Zero(),grid),"Save projected grid as PNG/YAML");
  auto map=saved_map(maps);require(map.info["available"].asBool()&&map.info["width"].asUInt()==100,"Published grid is readable by native panel");
  Tasks t(maps);t.hooks.mode=[](auto){throw std::runtime_error("start failed");};
  rejects([&]{t.request_mode("navigation");},"Propagate process startup failure");
  require(!t.switching&&t.mode=="idle","Startup failure must not lock mode switching");
  t.mode="navigation";t.localized=t.hardware_ready=t.nav_server=t.stack_ready=t.serial_ready=true;
  t.cloud_at=t.odom_at=t.pose_at=t.serial_at=10;t.pose["yaw"]=0;
  Json::Value goal;goal["x"]=1.;goal["y"]=1.;goal["map_revision"]=map.info["revision"];
  t.hooks.plan=[](auto,const auto&){throw std::runtime_error("planner unavailable");};
  rejects([&]{t.request_plan(goal,10);},"Propagate planning transport failure");
  require(t.plan["status"].asString()=="failed","Failed plan must not remain pending");
  t.hooks.goal=[](auto,const auto&){throw std::runtime_error("lease unavailable");};
  t.request_goal(goal,10);
  require(t.nav_status=="waiting"&&t.goal_pending&&!t.active(),"Accepted destination survives initial control transport failure");
  t.hardware(true,0);t.nav_status="active";t.resume=goal;t.hardware(true,1);
  require(t.nav_status=="canceling"&&!t.resume,"Authority epoch zero is valid: 0 -> 1 must cancel and revoke deferred resume");
}

void serial_recovery(){
  using namespace hw_runtime::serial;
  auto echo=frame(0x81,1,controls(1,{0,0,0,17}).substr(8,16));Controller c;c.receive(echo,10);
  Json::Value stop;stop["action"]="stop";stop["seq"]=1;
  for(int i=0;i<64;++i){stop["client"]="page-"+std::to_string(i);c.manual(stop,10+i*.001);}
  auto lease=c.acquire(10.1);Json::Value command;command["lease"]=lease;command["seq"]=1;command["forward"]=.3;command["left"]=0.;c.navigation_command(command,10.1);
  stop["client"]="new-page";stop["strength"]=Json::Value();stop["spin_rate"]="invalid input";
  c.manual(stop,10.11);require(c.lease.empty()&&c.desired(10.11)[0]==0,"New-page emergency stop must bypass capacity and unused movement fields");
  rejects([&]{c.acquire(10.115,c.manual_epoch-1);},"An acquire issued before manual takeover must not retake control afterward");
  require(c.clients.size()<=64,"Client LRU stays bounded");
  lease=c.acquire(10.12);command["lease"]=lease;command["seq"]=1;c.navigation_command(command,10.12);
  c.receive(echo,10.7);require(c.lease.empty(),"Orphan lease expires despite healthy MCU status");
  auto next=c.acquire(10.7);require(next!=lease,"New supervisor can acquire after lease TTL");
  command["seq"]=2;rejects([&]{c.navigation_command(command,10.71);},"Late commands from an expired lease cannot renew authority");
  command["lease"]=next;command["seq"]=1;c.navigation_command(command,10.72);require(c.desired(10.72)[0]>0,"New lease can start sequence at one");
  c.tick(100);require(c.clients.size()<64,"Idle clients expire");
}
void automatic_retry(const fs::path& root){
  auto maps=root/"retry-map";save_grid(maps,100,100,.025,Point::Zero(),std::vector<int8_t>(10000,0));
  Tasks t(maps);t.mode="navigation";t.localized=t.hardware_ready=t.nav_server=t.stack_ready=t.serial_ready=true;t.pose["yaw"]=0;t.localization["state"]="tracking";
  auto fresh=[&](double at){t.cloud_at=t.odom_at=t.pose_at=t.serial_at=at;};fresh(10);
  Json::Value goal;goal["x"]=1.;goal["y"]=1.;goal["map_revision"]=saved_map(maps).info["revision"];t.resume=goal;t.goal=goal;t.goal_pending=true;t.nav_status="waiting";
  int attempts=0;t.hooks.plan=[&](auto,const auto&){++attempts;throw std::runtime_error("planner unavailable");};
  t.tick(10);require(t.resume&&attempts==1,"Resume must retain its goal on planner transport failure");
  t.tick(10.1);require(attempts==1,"Resume transport retry is backed off");
  t.hooks.plan=[&](auto,const auto&){++attempts;};fresh(15);t.tick(15);
  t.plan_result(t.plan_generation,{Eigen::Vector2d(1,1)});
  t.hooks.goal=[](auto,const auto&){throw std::runtime_error("UART lease busy");};t.tick(15.1);
  require(t.resume&&t.nav_status=="waiting","UART acquire failure cannot escape timer or discard the deferred goal");
  fresh(21);t.tick(21);t.plan_result(t.plan_generation,{Eigen::Vector2d(1,1)});
  t.hooks.goal=[](auto,const auto&){};t.tick(21.1);require(t.resume&&t.nav_status=="sending","Keep deferred target until asynchronous goal acceptance");
  t.goal_response(t.goal_generation,false);require(t.resume.has_value(),"Async rejection preserves retry target");
  fresh(27);t.tick(27);t.plan_result(t.plan_generation,{Eigen::Vector2d(1,1)});t.tick(27.1);t.goal_response(t.goal_generation,true);
  require(!t.resume&&t.nav_status=="active","Accepted retry clears only its own deferred target");
  t.hooks.cancel=[](){throw std::runtime_error("action disconnected");};t.cancel();
  require(!t.enabled(27.1)&&t.nav_status=="canceling","Cancel transport failure must still disable motion");
}

void persistent_destination(const fs::path& root){
  auto maps=root/"persistent-map";save_grid(maps,100,100,.025,Point::Zero(),std::vector<int8_t>(10000,0));
  Tasks t(maps);t.mode="navigation";t.localized=t.hardware_ready=t.nav_server=t.stack_ready=t.serial_ready=true;
  t.pose["yaw"]=0;t.localization["state"]="tracking";t.hardware(true,1);
  auto fresh=[&](double at){t.cloud_at=t.odom_at=t.pose_at=t.serial_at=at;};fresh(10);
  Json::Value goal;goal["x"]=1.;goal["y"]=1.;goal["map_revision"]=saved_map(maps).info["revision"];
  int goals=0,plans=0;t.hooks.goal=[&](auto,const auto&){++goals;};t.hooks.plan=[&](auto,const auto&){++plans;};t.hooks.cancel=[]{};
  t.request_goal(goal,10);t.goal_response(t.goal_generation,true,10);
  for(int i=0;i<15;++i){
    double now=11+i*3;fresh(now);auto previous=t.goal_generation;
    t.goal_result(previous,"failed","Blocked MPC rollout",now);
    require(t.goal_pending&&t.nav_status=="waiting"&&!t.enabled(now),"Controller abort pauses the mission and disables output");
    rejects([&]{t.request_goal(goal,now);},"Waiting mission cannot be replaced without canceling");
    t.tick(now+.1);require(plans==i,"Retries must not busy-loop while blocked");
    fresh(now+1.1);t.tick(now+1.1);t.plan_result(t.plan_generation,{Eigen::Vector2d(1,1)});t.tick(now+1.2);
    require(t.goal_generation>previous,"Each retry owns a fresh action generation");
    t.goal_result(previous,"succeeded","Obsolete result",now+1.25);require(t.goal_pending,"Old action result cannot complete current mission");
    t.goal_response(t.goal_generation,true,now+1.3);
    require(t.goal_pending&&t.nav_status=="active","Accepted retry remains a pending destination until actual arrival");
  }
  require(goals==16&&t.retry_count>=15,"Repeated failures beyond progress timeout never discard the target");
  t.goal_result(t.goal_generation,"succeeded","Arrived",60);fresh(1000);t.tick(1000);
  require(!t.goal_pending&&!t.resume&&goals==16,"Arrival permanently completes the mission");
  t.request_goal(goal,1000);t.goal_response(t.goal_generation,true,1000);auto old=t.goal_generation;
  t.goal_result(old,"failed","Blocked",1000.1);t.cancel(false,1000.2);fresh(2000);t.tick(2000);
  t.goal_response(old,true,2000);require(!t.goal_pending&&!t.resume&&t.nav_status=="canceled","Explicit cancel while waiting defeats late responses and future retry");
  t.request_goal(goal,2000);t.goal_response(t.goal_generation,true,2000);
  t.hardware(false,1);require(t.goal_pending&&t.resume,"Missing actual feedback preserves destination with output disabled");
  t.goal_result(t.goal_generation,"canceled","Paused",2000.1);t.hardware(true,1);
  t.hooks.plan=[](auto,const auto&){throw std::runtime_error("No path");};fresh(2002);t.tick(2002);
  require(t.goal_pending&&t.resume,"A failed replan preserves destination indefinitely");
  t.hardware(false,1,true);t.hardware(true,1);fresh(3000);t.tick(3000);
  require(!t.goal_pending&&!t.resume,"Explicit hardware revoke terminates the destination even without an epoch change");
  t.hooks.plan=[&](auto,const auto&){++plans;};t.request_goal(goal,3000);auto canceled=t.goal_generation;
  t.pause("No response",3000.1);fresh(3004);t.tick(3004);
  require(t.goal_generation!=canceled&&t.goal_pending&&t.nav_status=="waiting","Unacknowledged cancellation is bounded without ending mission");
  t.goal_response(canceled,true,3004);require(t.nav_status=="waiting","Late acceptance after cancellation timeout cannot re-arm an old goal");
  t.cancel(false,3004.1);fresh(4000);t.request_goal(goal,4000);t.goal_response(t.goal_generation,true,4000);
  fs::rename(maps/"site.yaml",maps/"site-unavailable.yaml");t.tick(4000.1);fresh(4004);t.tick(4004);
  require(t.goal_pending&&t.nav_status=="waiting","Missing map must not block cancellation deadlines or discard destination");
  t.cancel(false,4004.1);require(!t.goal_pending&&t.nav_status=="canceled","Waiting for map must remain explicitly cancelable");
  fs::rename(maps/"site-unavailable.yaml",maps/"site.yaml");fresh(5000);t.request_goal(goal,5000);t.goal_response(t.goal_generation,true,5000);
  save_grid(maps,100,100,.025,Point::Zero(),std::vector<int8_t>(10000,0));t.tick(5000.1);
  require(!t.goal_pending&&!t.resume,"Changed map terminates the old coordinate target");
}
void map_snapshot_and_reload(const fs::path& root,const fs::path& source,const fs::path& self){
  auto runtime=root/"managed",maps=root/"managed-map";fs::create_directories(runtime/"shaoshao/web_panel");
  auto robot=runtime/"shaoshao/web_panel/robot.yaml";fs::copy_file(source/"web_panel/robot.yaml",robot);
  save_grid(maps,100,100,.025,Point::Zero(),std::vector<int8_t>(10000,0));
  Cloud points;for(int i=0;i<1200;++i)points.emplace_back((i%30)*.02,(i/30)*.02,.2+(i%3)*.1);save_pcd(maps/"site.pcd",points);
  auto child=Process::spawn({self.string(),"--role","mapping"},root/"fake-stack.log");
  std::this_thread::sleep_for(std::chrono::milliseconds(80));Json::Value session;session["pid"]=child.pid();session["start_ticks"]=Json::UInt64(child.start());session["maps_dir"]=maps.string();write_json(runtime/"mapping-session.json",session);
  Manager manager(runtime,maps);bool export_fresh=false;
  manager.trigger=[&](const std::string& name){
    if(name=="/map_save"){for(auto& p:points)p.z()+=.001;save_pcd(maps/"site.pcd",points);}
    else if(name=="/hw/export_grid"&&export_fresh)save_grid(maps,100,100,.025,Point::Zero(),std::vector<int8_t>(10000,0));
    else if(name=="/hw/finalize_index"){IndexWriter writer(maps/"keyframes");writer.add(points,Pose::Identity(),1);writer.finalize(maps/"site.pcd",points);}
  };
  rejects([&]{manager.stop_mapping();},"An old 2D export cannot be accepted with a new PCD");
  require(manager.mapping_running()&&!manager.saved,"Keep mapping alive after stale export failure");
  export_fresh=true;manager.stop_mapping();require(manager.saved&&fs::exists(maps/"bundle.json"),"Consistent save writes an asset hash bundle");
  auto bundle=read_json(maps/"bundle.json");require(bundle["pcd_sha256"].asString()==sha256(maps/"site.pcd"),"Bundle binds exact PCD snapshot");
  atomic_write(maps/"site.yaml",read_text(maps/"site.yaml")+"# changed\n");rejects([&]{manager.start_navigation();},"Do not start navigation with a changed 2D asset");
  auto changed=YAML::LoadFile(robot.string());changed["navigation"]["max_speed_mps"]=.1;changed["navigation"]["mpc"]["tau_x_s"]=.7;write_yaml(robot,changed);
  manager.reload_config();Config snapshot(robot_configuration(runtime));
  require(manager.config.speed==.1&&snapshot.speed==.1&&snapshot.tau_x==.7,"Mode reload publishes one validated configuration snapshot");
  changed["navigation"]["max_speed_mps"]=-1.;write_yaml(robot,changed);rejects([&]{manager.reload_config();},"Invalid replacement config must be rejected");
  require(Config(robot_configuration(runtime)).speed==.1,"Rejected reload must preserve the last valid snapshot");
}
int main(int argc,char** argv){
  if(argc==3&&std::string(argv[1])=="--role"){
    static volatile std::sig_atomic_t stopped=0;std::signal(SIGINT,[](int){stopped=1;});
    while(!stopped)std::this_thread::sleep_for(std::chrono::milliseconds(10));return 0;
  }
  if(argc!=2){std::cerr<<"Provide repository root\n";return 2;}auto root=fs::temp_directory_path()/("hw-native-"+std::to_string(getpid()));fs::create_directories(root);
  try{localization();indices(root);uart();configurations(root,fs::absolute(argv[1]));processes(root);tasks(root);serial_recovery();automatic_retry(root);persistent_destination(root);map_snapshot_and_reload(root,fs::absolute(argv[1]),fs::absolute(argv[0]));fs::remove_all(root);std::cout<<"Native localization, index, UART, configuration, process and task tests passed\n";}
  catch(const std::exception& e){std::cerr<<e.what()<<"; evidence: "<<root<<'\n';return 1;}
}
