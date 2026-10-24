#pragma once
#include "feedback.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace hw_omni {
struct AuthoritySample {
  double stamp=0;uint64_t sequence=0,authority_epoch=0;bool control_allowed=false;
  std::string frame="navcar_center";
};
class AuthorityGate {
public:
  explicit AuthorityGate(double timeout=.15):timeout_(timeout){if(!std::isfinite(timeout)||timeout<=0||timeout>.2)throw std::invalid_argument("Invalid authority timeout");}
  SampleUpdate update(const AuthoritySample& value,double now){
    if(sample_&&(value.authority_epoch<sample_->authority_epoch||(value.authority_epoch==sample_->authority_epoch&&
        (value.sequence<=sample_->sequence||value.stamp<=sample_->stamp))))return SampleUpdate::replayed;
    if(!std::isfinite(now)||!std::isfinite(value.stamp)||value.frame!="navcar_center"||now-value.stamp<-.05||now-value.stamp>timeout_){invalid_=true;return SampleUpdate::invalid;}
    sample_=value;invalid_=false;return SampleUpdate::accepted;
  }
  bool ready(double now)const{return sample_&&!invalid_&&sample_->control_allowed&&std::isfinite(now)&&now-sample_->stamp>=-.05&&now-sample_->stamp<=timeout_;}
  const std::optional<AuthoritySample>& sample()const{return sample_;}
private:double timeout_;bool invalid_=false;std::optional<AuthoritySample> sample_;
};
struct MotionSample {
  double stamp=0;std::string frame="navcar_odom",child="navcar_center";
  Eigen::Vector3d position=Eigen::Vector3d::Zero(),velocity=Eigen::Vector3d::Zero(),omega=Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation=Eigen::Quaterniond::Identity();
  Eigen::Matrix<double,6,6> covariance=Eigen::Matrix<double,6,6>::Zero();
};
class MotionGate {
public:
  explicit MotionGate(double timeout=.15,double velocity_std=.5):timeout_(timeout),variance_(velocity_std*velocity_std){
    if(!std::isfinite(timeout)||timeout<=0||timeout>.15||!std::isfinite(velocity_std)||velocity_std<=0||velocity_std>2)throw std::invalid_argument("Invalid LIO motion bounds");
  }
  SampleUpdate update(const MotionSample& value,double now){
    if(std::isfinite(value.stamp)&&last_seen_&&value.stamp<=*last_seen_)return SampleUpdate::replayed;
    if(!std::isfinite(now)||!std::isfinite(value.stamp)||now-value.stamp<-.05||now-value.stamp>timeout_){invalid_=true;return SampleUpdate::invalid;}
    last_seen_=value.stamp;
    auto bad=[&]{invalid_=true;return SampleUpdate::invalid;};
    if(value.frame!="navcar_odom"||value.child!="navcar_center"||!value.position.allFinite()||!value.velocity.allFinite()||!value.omega.allFinite()||
       !value.orientation.coeffs().allFinite()||std::abs(value.orientation.norm()-1)>.001||value.velocity.norm()>3||value.omega.norm()>30||
       !value.covariance.allFinite()||(value.covariance-value.covariance.transpose()).cwiseAbs().maxCoeff()>1e-6||
       value.covariance(0,0)>variance_||value.covariance(1,1)>variance_||value.covariance(5,5)>1)return bad();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6>> eigen(value.covariance);
    if(eigen.info()!=Eigen::Success||eigen.eigenvalues().minCoeff()< -1e-8)return bad();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> planar(value.covariance.topLeftCorner<2,2>());
    if(planar.info()!=Eigen::Success||planar.eigenvalues().maxCoeff()>variance_)return bad();
    sample_=value;invalid_=false;return SampleUpdate::accepted;
  }
  bool ready(double now)const{return sample_&&!invalid_&&std::isfinite(now)&&now-sample_->stamp>=-.05&&now-sample_->stamp<=timeout_;}
  const std::optional<MotionSample>& sample()const{return sample_;}
private:double timeout_,variance_;bool invalid_=false;std::optional<double> last_seen_;std::optional<MotionSample> sample_;
};
inline Eigen::Quaterniond rotation_increment(const Eigen::Vector3d& omega,double dt){
  double angle=omega.norm()*dt;if(std::abs(angle)<1e-12)return Eigen::Quaterniond::Identity();
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle,omega.normalized()));
}
inline double heading(const Eigen::Quaterniond& q){return std::atan2(2*(q.w()*q.z()+q.x()*q.y()),1-2*(q.y()*q.y()+q.z()*q.z()));}
struct MotionProjection {Eigen::Vector2d position,velocity;double yaw=0,omega=0;};
// Pose and LIO twist may have different sample times. Rotate the body-frame
// velocity at its own sample heading, then extrapolate the pose by at most the
// already-validated freshness window. Chassis spin is never a radar yaw rate.
inline MotionProjection project_motion(const MotionSample& motion,const Eigen::Vector3d& map_position,
    const Eigen::Quaterniond& map_orientation,double pose_stamp,double now){
  if(!map_position.allFinite()||!map_orientation.coeffs().allFinite()||std::abs(map_orientation.norm()-1)>.001||
     !std::isfinite(pose_stamp)||!std::isfinite(now)||now-pose_stamp<-.05||now-pose_stamp>.15||now-motion.stamp<-.05||now-motion.stamp>.15)
    throw std::invalid_argument("Invalid/stale map pose or LIO projection");
  auto sample_orientation=map_orientation*rotation_increment(motion.omega,motion.stamp-pose_stamp);
  auto current_orientation=map_orientation*rotation_increment(motion.omega,std::max(0.,now-pose_stamp));
  Eigen::Vector3d velocity=sample_orientation*motion.velocity;
  MotionProjection out;out.velocity=velocity.head<2>();out.position=(map_position+velocity*std::max(0.,now-pose_stamp)).head<2>();
  out.yaw=heading(current_orientation);out.omega=motion.omega.z();return out;
}
}
