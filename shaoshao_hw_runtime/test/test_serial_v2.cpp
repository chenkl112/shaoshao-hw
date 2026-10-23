#include "shaoshao_runtime/serial.hpp"
#include <iostream>
using namespace hw_runtime;
using namespace hw_runtime::serial;
void check(bool value,const char* why){if(!value)throw std::runtime_error(why);}
std::string state(uint16_t seq,Values v){std::string payload;for(float f:v){uint32_t b=std::bit_cast<uint32_t>(f);for(int i=0;i<4;++i)payload.push_back(char((b>>(i*8))&255));}return frame(0x81,seq,payload);}
int main(int argc,char** argv){try{
  check(argc==2,"Expected source root");auto path=fs::path(argv[1])/"web_panel/serial.navcar.example.yaml";
  auto temp=fs::temp_directory_path()/("navcar-v2-test-"+std::to_string(getpid())+".yaml");auto yaml=YAML::LoadFile(path.string());yaml["serial_port"]="/dev/test-only";write_yaml(temp,yaml);Settings settings(temp);fs::remove(temp);
  check(settings.v2&&settings.forward==1&&settings.left==1&&settings.rot==1,"V2 must use SI units without legacy multiplier");
  yaml["forward_units_per_mps"]=10.;write_yaml(temp,yaml);bool rejected=false;try{Settings wrong(temp);}catch(const std::exception&){rejected=true;}fs::remove(temp);check(rejected,"V2 must reject legacy scale=10 configuration");
  check(controls(0,{1,0,0,0})==std::string("\xA5\x5A\x01\x01\x00\x00\x10\x00\x00\x00\x80\x3F\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x4B\x6C",26),"Official V2 control CRC vector");
  float nan=std::numeric_limits<float>::quiet_NaN();Parser parser;auto packets=parser.feed(state(7,{nan,nan,nan,23}),1);check(packets.size()==1&&!packets[0].velocity_valid()&&packets[0].yaw_valid(),"NaN wheel feedback must invalidate velocity while preserving yaw");
  packets=parser.feed(state(7,{.1,.2,.3,nan}),1.01);check(packets.size()==1&&packets[0].velocity_valid()&&!packets[0].yaw_valid(),"Independent IMU invalidity must be visible");
  Session session;session.receive(state(65535,{0,0,0,17}),1);check(session.sequence&&*session.sequence==0,"Command sequence wraps at startup");session.receive(state(65535,{.1,.2,.3,18}),1.01);check(session.reception_seq==2&&session.state->values[0]==.1f,"Repeated command sequence must not discard new measurements");
  auto first=session.poll({0,0,0,17},1.01);check(first==controls(0,{0,0,0,17}),"First command uses state sequence plus one");check(session.poll({.1,0,0,17},1.061)==first,"ACK retry must preserve exact bytes");session.receive(frame(0x80,0,std::string(1,'\1')),1.07);check(session.pending.empty()&&*session.sequence==1,"Duplicate ACK completes transaction without treating measurements as targets");
  Controller c(settings);c.receive(state(12,{.1,.2,.3,17}),10);auto lease=c.acquire(10);auto epoch=c.manual_epoch;
  Json::Value cmd;cmd["lease"]=lease;cmd["seq"]=1;cmd["forward"]=.2;cmd["left"]=-.1;c.navigation_command(cmd,10);
  auto native=c.desired(10);check(std::abs(native[0]-.2)<1e-6&&std::abs(native[1]-.1)<1e-6,"V2 forward/left -> forward/right in SI");
  Json::Value spin;spin["enabled"]=true;spin["rate"]=.7;spin["client"]="operator";spin["seq"]=1;c.set_spin(spin,10.01);
  check(c.lease==lease&&c.manual_epoch==epoch&&c.desired(10.01)[2]>.69,"Independent spin must preserve translation lease and use CCW positive sign");
  cmd["seq"]=2;c.navigation_command(cmd,10.02);check(c.desired(10.02)[2]>.69,"Translation heartbeat must not clear spin");
  check(c.desired(10.215)[0]>.19&&c.desired(10.215)[2]==0,"Spin expires independently of translation");
  spin["seq"]=2;c.set_spin(spin,10.22);check(c.desired(10.23)[0]==0&&c.desired(10.23)[2]>.69,"Spin heartbeat must not renew translation");
  auto report=c.state(10.23);check(report["feedback_targets"].isNull()&&report["yaw_actual_deg"].asDouble()==17,"V2 UI separates measurements from targets");
  Json::Value yaw;yaw["angle_deg"]=45.;yaw["client"]="operator";yaw["seq"]=3;c.set_yaw(yaw,10.24);c.receive(state(13,{0,0,0,18}),10.25);check(c.desired(10.25)[3]==45,"Measured yaw must not overwrite the held target");
  c.receive(state(13,{nan,nan,nan,19}),10.26);check(!c.measurements_ready()&&c.connected(10.26)&&!c.state(10.26)["error"].asString().empty(),"Invalid measurements must be distinct from link loss");check(c.desired(10.26)[0]==0&&c.desired(10.26)[2]==0,"Invalid velocity must stop outgoing motion immediately");
  c.receive(state(13,{0,0,0,19}),10.27);check(c.measurements_ready(),"Fresh wheel feedback restores validity");
  lease=c.acquire(10.28);cmd["lease"]=lease;cmd["seq"]=1;c.navigation_command(cmd,10.28);
  Json::Value ros_spin;ros_spin["lease"]=lease;ros_spin["seq"]=1;ros_spin["omega"]=-.6;c.spin_command(ros_spin,10.29);check(c.desired(10.29)[2]<-.59,"ROS spin uses its own sequence and positive-CCW SI mapping");
  spin["seq"]=4;spin["rate"]=.2;c.set_spin(spin,10.295);
  c.spin_command(ros_spin,10.296);check(std::abs(c.desired(10.296)[2]-.2)<1e-6,"A replayed ROS spin must not regain authority after HTTP spin override");
  ros_spin["seq"]=2;c.spin_command(ros_spin,10.297);
  c.release(lease);check(c.desired(10.3)[2]==0,"Releasing a ROS spin authorization must stop it");
  std::cout<<"V2 official vectors, SI signs, independent validity, yaw hold, command sync and independent spin passed\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
