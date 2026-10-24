#include "hw_omni_controller/obstacles.hpp"
#include "hw_omni_controller/cloud.hpp"
#include "hw_omni_controller/relay.hpp"
#include <iostream>
#include <stdexcept>

void require(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
template<class F>void rejects(F action,const char* message){bool failed=false;try{action();}catch(const std::exception&){failed=true;}require(failed,message);}
hw_obstacles::Detection detection(double x,double y=0){
  return {hw_obstacles::Point(x,y),{{-.1,-.1},{.1,-.1},{.1,.1},{-.1,.1}}};
}
void detector_and_shapes(){
  using namespace hw_obstacles;
  std::vector<CloudPoint> wall,points;
  for(int y=0;y<25;++y)for(int z=0;z<15;++z)wall.emplace_back(2.,-1+y/12.,.2+z/14.);
  points=wall;
  for(int x=0;x<10;++x)for(int y=0;y<10;++y)for(double z:{.3,.6,.9})points.emplace_back(.8+x*.2/9,.2+y*.2/9,z);
  for(int i=0;i<100;++i)points.emplace_back(-2+i*.04,0,0);
  auto nearest=[&](const CloudPoint& p){double d=100;for(const auto& w:wall)d=std::min(d,(w-p).norm());return d;};
  Detector detector;auto found=detector.detect(points,Point::Zero(),0,nearest);
  require(found.size()==1,"Detect new object while removing static structure and ground");
  require((found[0].center-Point(.9,.3)).norm()<.02,"Dynamic object centroid");
  require(found[0].footprint.size()>=4,"Nondegenerate object footprint");
  require(detector.detect(wall,Point::Zero(),0,nearest).empty(),"Static map must not become a dynamic target");
  auto shape=hull({Point(0,0),Point(0,.2)});
  require(shape.size()>=4,"Thin returns must retain a padded footprint");
  rejects([&]{detector.detect(std::vector<CloudPoint>(200001),Point::Zero(),0,nearest);},"Point budget enforcement");
}
void tracking(){
  using namespace hw_obstacles;
  Tracker tracker;tracker.update({detection(1.)},1.);
  auto first=tracker.predict();require(first.size()==1&&!first[0].confirmed,"Unconfirmed target retention");
  for(const auto& p:first[0].centers)require((p-Point(1,0)).norm()<1e-9,"Unconfirmed target remains in all future steps");
  for(int i=1;i<10;++i)tracker.update({detection(1+.05*i)},1+.1*i);
  auto moving=tracker.predict()[0];require(moving.confirmed,"Repeated observations confirm a track");
  require(std::abs(tracker.tracks()[0].state[2]-.5)<.08,"Actual target velocity estimate");
  require(moving.centers[10].x()>moving.centers[0].x()+.3,"Future obstacle displacement");
  tracker.update({},2.);require(tracker.predict()[0].id==moving.id,"Occluded tracks retain identity");
  require(tracker.predict()[0].centers.back().x()>tracker.predict()[0].centers.front().x(),"Occluded tracks retain motion prediction");
  tracker.update({},2.6);require(tracker.predict().empty(),"Expired tracks are cleared after a sampling gap");
  tracker.update({detection(0,-.5),detection(2,.5)},3.);
  for(int i=1;i<12;++i)tracker.update({detection(.1*i,-.5),detection(2-.1*i,.5)},3+.1*i);
  require(tracker.tracks().size()==2,"One-to-one target association");
  require(tracker.tracks()[0].state[2]>.8&&tracker.tracks()[1].state[2]<-.8,"Opposing target velocities");
  for(const auto& t:tracker.tracks()){
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigen(t.covariance);
    require(eigen.eigenvalues().minCoeff()>-1e-10,"Joseph covariance stays positive semidefinite");
  }
  rejects([&]{tracker.update({detection(50)},4.);},"Old observations cannot advance tracking");
  auto assignment=associate({{.1,.2},{.11,.9}},.8);
  require(assignment==std::vector<int>({1,0}),"Hungarian assignment minimizes joint cost");
}
void clouds(){
  using namespace hw_obstacles;
  for(bool big:{false,true}){
    // Organized 2x2 float64 cloud with per-point and per-row padding.
    std::vector<uint8_t> bytes(144,0);
    std::array<FloatField,3> fields{{{0,8},{8,8},{16,8}}};
    for(size_t y=0;y<2;++y)for(size_t x=0;x<2;++x)for(size_t axis=0;axis<3;++axis){
      double value=10*y+3*x+axis+.5;uint64_t bits=std::bit_cast<uint64_t>(value);
      for(size_t b=0;b<8;++b)bytes[y*72+x*32+axis*8+b]=static_cast<uint8_t>(bits>>(8*(big?7-b:b)));
    }
    auto points=decode_cloud(bytes,2,2,32,72,big,fields);
    require(points.size()==4&&std::abs(points[3].z()-15.5)<1e-10,"Endian-aware organized cloud decoding");
    rejects([&]{decode_cloud(bytes,2,2,32,63,big,fields);},"Reject truncated cloud row strides");
    rejects([&]{decode_cloud(std::vector<uint8_t>(10),2,2,32,72,big,fields);},"Reject short cloud buffer");
    fields[0].offset=std::numeric_limits<size_t>::max();
    rejects([&]{decode_cloud(bytes,2,2,32,72,big,fields);},"Reject overflowing cloud field offsets");
  }
}
hw_omni::ChassisSample sample(double stamp=1,uint64_t seq=1,uint64_t epoch=1,bool allowed=true){
  hw_omni::ChassisSample s;s.stamp=stamp;s.sequence=seq;s.authority_epoch=epoch;s.control_allowed=allowed;
  s.vx=.08;s.omega=4;s.gimbal_yaw=.3;return s;
}
void feedback(){
  using namespace hw_omni;
  FeedbackGate gate;require(!gate.ready(1),"No feedback must block motion");
  require(gate.update(sample(),1)==SampleUpdate::accepted&&gate.ready(1.1),"Measured feedback and independent spin accepted");
  require(gate.update(sample(1.1),1.1)==SampleUpdate::replayed&&!gate.ready(1.16),"Replay does not renew freshness");
  auto bad=sample(1.12,2);bad.vx=std::numeric_limits<double>::quiet_NaN();
  require(gate.update(bad,1.12)==SampleUpdate::invalid&&!gate.ready(1.12),"Invalid feedback blocks output");
  require(gate.update(sample(1.13),1.13)==SampleUpdate::replayed&&!gate.ready(1.13),"Replay cannot clear an invalid-feedback fault");
  require(gate.update(sample(1.14,3),1.14)==SampleUpdate::accepted&&gate.ready(1.14),"New real sample clears fault");
  gate.update(sample(1.15,0,2,false),1.15);require(!gate.ready(1.15),"Hardware revoke stops output");
  gate.update(sample(1.16,0,3,true),1.16);require(gate.ready(1.16),"New authority epoch allows sequence restart");
  gate.update(sample(1.17,99,1),1.17);require(gate.sample()->authority_epoch==3,"Old authority cannot regain control");
}
hw_omni::AuthoritySample authority(double stamp=1,uint64_t seq=1,uint64_t epoch=1,bool allowed=true){return {stamp,seq,epoch,allowed,"navcar_center"};}
hw_omni::MotionSample motion(double stamp=1,double vx=.08){hw_omni::MotionSample m;m.stamp=stamp;m.velocity.x()=vx;return m;}
hw_omni::CommandRelay relay(){
  using namespace hw_omni;CommandRelay r;
  r.update_authority(authority(),1);r.update_motion(motion(),1);r.update_session({1,"lease-A",1,true},1);
  r.raw=U(.08,.03);r.raw_stamp=1;r.pose_stamp=1;r.localization_stamp=1;r.localized=true;r.localization_speed=.5;
  r.prediction_valid=true;r.prediction_stamp=1;r.grid_stamp=1;r.grid.width=200;r.grid.height=200;
  r.grid.resolution=.025;r.grid.origin=U(-2.5,-2.5);r.grid.cells.assign(40000,0);return r;
}
void relay_gates(){
  using namespace hw_omni;
  auto r=relay();auto output=r.step(1.02);require(output.moving&&(output.velocity-U(.08,.03)).norm()<1e-12,"Relay uses FAST-LIO motion and independent authority");
  r.update_authority(authority(1.1,2),1.1);require(!r.step(1.16).moving&&r.step(1.16).reason.find("FAST-LIO")!=std::string::npos,"Fresh authority cannot hide stale LIO velocity");
  r=relay();r.update_authority(authority(1.03,0,2,false),1.03);require(!r.step(1.03).moving,"Hardware takeover stops relay immediately");
  r.update_authority(authority(1.04,0,3,true),1.04);require(!r.step(1.04).moving,"Re-enable cannot reuse old-epoch lease");
  r.update_session({1.04,"lease-B",3,true},1.04);require(!r.step(1.04).moving,"New session cannot reuse a previous MPC command");
  r.raw_stamp=1.04;require(r.step(1.04).moving,"Fresh session and command can resume explicitly");
  r=relay();r.prediction_valid=false;require(!r.step(1.02).moving,"Missing predictions block output");
  r=relay();auto bad=motion(1.01);bad.velocity.x()=std::numeric_limits<double>::quiet_NaN();r.update_motion(bad,1.01);
  r.update_motion(motion(1.02),1.02);require(!r.step(1.02).moving,"LIO recovery requires a new MPC command");
  r=relay();r.update_session({1.01,"lease-A",1,false},1.01);require(!r.step(1.02).moving,"Task cancellation emits zero");
  r=relay();r.raw=U(.01,0);r.update_motion(motion(1.02,.3),1.02);
  r.grid.cells[100*200+108]=253;require(!r.step(1.02).moving,"Braking guard uses LIO speed, not only a small target");
  r=relay();Obstacle obstacle;obstacle.footprint={U(-.1,-.1),U(.1,-.1),U(.1,.1),U(-.1,.1)};
  for(int i=0;i<=30;++i)obstacle.centers.emplace_back(.05,.5-i*.1);
  r.obstacles={obstacle};require(!r.step(1.02).moving,"Relay checks future obstacle position in braking corridor");
  r=relay();r.grid.cells[110*200+100]=255;require(!r.step(1.02).moving,"Relay checks unknown cells under the full body");
  r=relay();require(r.step(1.25).lease=="lease-A","Stale task heartbeat first sends a bounded zero");
  require(r.step(1.5).lease.empty(),"An orphan task stops renewing UART lease with zeros");
}
int main(){
  try{detector_and_shapes();tracking();clouds();feedback();relay_gates();std::cout<<"All native obstacle, feedback and command-relay tests passed\n";}
  catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
