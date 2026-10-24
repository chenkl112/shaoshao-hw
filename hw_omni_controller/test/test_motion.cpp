#include "hw_omni_controller/ros_samples.hpp"
#include <iostream>
using namespace hw_omni;
void require(bool value,const char* why){if(!value)throw std::runtime_error(why);}
int main(){try{
  nav_msgs::msg::Odometry raw;raw.header.frame_id="navcar_odom";raw.child_frame_id="navcar_imu";raw.header.stamp.sec=1;
  raw.pose.pose.orientation.w=1;raw.twist.twist.linear.x=.2;raw.twist.twist.angular.z=2;
  for(int i=0;i<6;++i)raw.twist.covariance[i*6+i]=.001;
  auto center=center_odometry(raw,Eigen::Vector3d(.02,-.03,0));auto m=motion_sample(center);
  require(center.header.stamp==raw.header.stamp&&center.child_frame_id=="navcar_center","Center shift preserves FAST-LIO sample time");
  require((m.velocity-Eigen::Vector3d(.26,.04,0)).norm()<1e-12,"Center velocity includes omega cross IMU lever arm");
  require(std::abs(m.covariance(0,5)-.00003)<1e-12&&std::abs(m.covariance(1,5)-.00002)<1e-12,"Shifted covariance includes velocity/gyro correlation");
  MotionGate gate;require(gate.update(m,1.01)==SampleUpdate::accepted&&gate.ready(1.1),"Fresh fused LIO velocity is accepted");
  auto bad=m;bad.stamp=1.02;bad.velocity.x()=std::numeric_limits<double>::quiet_NaN();require(gate.update(bad,1.02)==SampleUpdate::invalid&&!gate.ready(1.02),"Nonfinite LIO speed stops motion");
  require(gate.update(m,1.03)==SampleUpdate::replayed&&!gate.ready(1.03),"Replayed valid motion cannot clear a newer fault");
  m.stamp=1.04;require(gate.update(m,1.04)==SampleUpdate::accepted&&gate.ready(1.04),"New LIO sample restores readiness");
  bad=m;bad.stamp=1.05;bad.covariance(0,0)=.26;require(gate.update(bad,1.05)==SampleUpdate::invalid,"Uncertain planar velocity stops output");
  bad=m;bad.stamp=1.055;bad.covariance(0,0)=bad.covariance(1,1)=.15;bad.covariance(0,1)=bad.covariance(1,0)=.14;require(gate.update(bad,1.055)==SampleUpdate::invalid,"Correlated planar uncertainty is bounded along its principal axis");
  bad=m;bad.stamp=1.06;bad.covariance(0,0)=.001;bad.covariance(0,1)=bad.covariance(1,0)=1;require(gate.update(bad,1.06)==SampleUpdate::invalid,"Indefinite motion covariance is rejected");
  m.stamp=1.07;gate.update(m,1.07);require(!gate.ready(1.221),"LIO data cannot be renewed by control/authority heartbeats");
  bad=m;bad.stamp=1.08;bad.child="navcar_chassis";require(gate.update(bad,1.08)==SampleUpdate::invalid,"Body wheel feedback cannot masquerade as lidar motion");
  auto yaw90=Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d::UnitZ()));m.stamp=2;m.omega.setZero();m.velocity={.3,.1,0};
  auto projected=project_motion(m,Eigen::Vector3d::Zero(),yaw90,2,2.05);
  require((projected.velocity-Eigen::Vector2d(-.1,.3)).norm()<1e-12,"Map velocity uses lidar sample orientation");
  require((projected.position-Eigen::Vector2d(-.005,.015)).norm()<1e-12,"Map pose extrapolation uses LIO speed");
  m.omega.z()=M_PI/2/.1;projected=project_motion(m,Eigen::Vector3d::Zero(),yaw90,2.1,2.1);
  require((projected.velocity-Eigen::Vector2d(.3,.1)).norm()<1e-10,"Sample-time heading differs from current heading during rotation");
  AuthorityGate authority;require(authority.update({3,1,0,true,"navcar_center"},3)==SampleUpdate::accepted&&authority.ready(3.1),"Authority is independent of all velocity/gimbal fields");
  authority.update({3.02,0,1,false,"navcar_center"},3.02);require(!authority.ready(3.02),"Hardware revoke remains effective");
  authority.update({3.03,0,2,true,"navcar_center"},3.03);require(authority.ready(3.03),"New authority epoch permits counter restart");
  std::cout<<"LIO lever arm, frame/time alignment, covariance, replay, freshness and independent authority passed\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
