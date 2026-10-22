#include "shaoshao_runtime/configuration.hpp"
#include "shaoshao_runtime/index.hpp"
#include "shaoshao_runtime/pcd.hpp"
#include "shaoshao_runtime/policy.hpp"
#include "shaoshao_runtime/process.hpp"
#include "shaoshao_runtime/serial.hpp"
#include "shaoshao_runtime/tasks.hpp"
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
  Controller c;c.receive(frame(0x81,1,controls(1,{0,0,0,17}).substr(8,16)),10);auto lease=c.acquire(10);
  Json::Value command;command["lease"]=lease;command["seq"]=1;command["forward"]=.3;command["left"]=.4;c.navigation_command(command,10);
  auto native=c.desired(10);require(std::abs(native[0]-3)<1e-6&&std::abs(native[1]+4)<1e-6,"Apply UART multiplier exactly once");
  Json::Value manual;manual["action"]="stop";manual["strength"]=.1;manual["client"]="operator";manual["seq"]=1;c.manual(manual,10.01);
  command["seq"]=2;rejects([&]{c.navigation_command(command,10.02);},"Manual takeover revokes previous navigation lease");require(c.desired(10.02)[0]==0,"Stop command takes effect");
}
void configurations(const fs::path& root,const fs::path& source){
  auto runtime=root/"runtime";fs::create_directories(runtime);fs::create_directory_symlink(source,runtime/"shaoshao");configure_runtime(source,runtime);Config config(source/"web_panel/robot.yaml");
  auto maps=root/"new-map";fs::create_directories(maps);write_yaml(runtime/"mapping_runtime/fast_lio.yaml",mapping_configuration(source,maps,config));
  auto lio=YAML::LoadFile((runtime/"mapping_runtime/fast_lio.yaml").string());
  require(lio["/**"]["ros__parameters"]["mapping"]["extrinsic_R"][0].Scalar()=="1.0", "ROS rotation parameters must retain double_array type");
  atomic_write(maps/"site.yaml","image: site.png\nresolution: 0.025\n");auto frames=navigation_configuration(runtime,maps,config);
  require(std::abs(frames["center_offset"][0].asDouble()-.00318)<1e-8,"Preserve measured reference offset");auto nav=YAML::LoadFile((runtime/"navigation_runtime/nav2.yaml").string());
  require(nav["controller_server"]["ros__parameters"]["FollowPath"]["plugin"].as<std::string>()=="hw_omni_controller::Controller","Native configuration keeps FDDP");require(nav["local_costmap"]["local_costmap"]["ros__parameters"]["global_frame"].as<std::string>()=="map","Native configuration keeps map-frame predictions");
  require(read_text(runtime/"navigation_runtime/nav2.yaml").find("&1")==std::string::npos,"No ROS YAML anchors");
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
  rejects([&]{t.request_goal(goal,10);},"Propagate control lease failure");
  require(t.nav_status=="failed"&&!t.active(),"Failed goal must not remain active");
  t.hardware(true,0);t.nav_status="active";t.resume=goal;t.hardware(true,1);
  require(t.nav_status=="canceling"&&!t.resume,"Authority epoch zero is valid: 0 -> 1 must cancel and revoke deferred resume");
}
int main(int argc,char** argv){
  if(argc!=2){std::cerr<<"Provide repository root\n";return 2;}auto root=fs::temp_directory_path()/("hw-native-"+std::to_string(getpid()));fs::create_directories(root);
  try{localization();indices(root);uart();configurations(root,fs::absolute(argv[1]));processes(root);tasks(root);fs::remove_all(root);std::cout<<"Native localization, index, UART, configuration, process and task tests passed\n";}
  catch(const std::exception& e){std::cerr<<e.what()<<"; evidence: "<<root<<'\n';return 1;}
}
