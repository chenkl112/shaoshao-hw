#pragma once
#include "motion.hpp"
#include "model.hpp"

namespace hw_omni {
struct Session {
  double stamp=0;
  std::string lease;
  uint64_t authority_epoch=0;
  bool enabled=false;
};
struct RelayOutput {std::string lease,reason;uint64_t sequence=0;U velocity=U::Zero();bool moving=false;};

class CommandRelay {
public:
  AuthorityGate authority;
  MotionGate motion;
  Grid grid;
  std::vector<Obstacle> obstacles;
  Eigen::Vector3d position=Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation=Eigen::Quaterniond::Identity();
  U raw=U::Zero();
  double pose_stamp=0,raw_stamp=0,grid_stamp=0;
  double localization_stamp=0,localization_speed=0,prediction_stamp=0;
  double max_speed=.5,deceleration=.3,jerk=1.,radius=.317658;
  bool localized=false,prediction_valid=false;

  SampleUpdate update_authority(const AuthoritySample& value,double now){
    auto old=authority.sample();auto update=authority.update(value,now);
    if(update==SampleUpdate::invalid||(update==SampleUpdate::accepted&&(!value.control_allowed||(old&&old->authority_epoch!=value.authority_epoch)))){
      raw_stamp=0;enabled_at_=now;
    }return update;
  }
  SampleUpdate update_motion(const MotionSample& value,double now){
    auto update=motion.update(value,now);if(update==SampleUpdate::invalid){raw_stamp=0;enabled_at_=now;}return update;
  }

  bool update_session(const Session& value,double now){
    if(!fresh(now,value.stamp,.2)||value.lease.size()>128||value.lease.empty())return false;
    if(session_&&value.stamp<=session_->stamp)return false;
    if(!session_||value.lease!=session_->lease){sequence_=0;enabled_at_=value.stamp;}
    else if(value.enabled&&!session_->enabled)enabled_at_=value.stamp;
    session_=value;return true;
  }
  RelayOutput step(double now){
    RelayOutput out;
    if(!session_){out.reason="No navigation lease";return out;}
    out.lease=session_->lease;out.sequence=++sequence_;
    auto stop=[&](const std::string& why){out.reason=why;return out;};
    if(!fresh(now,session_->stamp,.2)||!session_->enabled){
      if(now-session_->stamp>.4)out.lease.clear(); // Do not renew an orphan UART lease forever with zeros.
      return stop("Navigation session inactive or stale");
    }
    if(!authority.ready(now))return stop("Hardware control authority unavailable");
    if(!motion.ready(now))return stop("FAST-LIO velocity unavailable or uncertain");
    if(authority.sample()->authority_epoch!=session_->authority_epoch)return stop("Hardware authority changed");
    if(!localized||!fresh(now,localization_stamp,.5)||localization_speed<=0)return stop("Map localization unavailable");
    if(!fresh(now,raw_stamp,.2)||raw_stamp<enabled_at_||!raw.allFinite())return stop("MPC command stale or predates the session");
    if(!fresh(now,pose_stamp,.15)||!position.allFinite()||!orientation.coeffs().allFinite())return stop("Lidar pose unavailable");
    if(!prediction_valid||!fresh(now,prediction_stamp,.3))return stop("Obstacle prediction unavailable");
    if(!fresh(now,grid_stamp,.5)||grid.cells.empty())return stop("Local collision grid unavailable");
    U target=raw;double limit=std::min(max_speed,localization_speed);
    if(target.norm()>limit)target*=limit/target.norm();
    MotionProjection projection;
    try{projection=project_motion(*motion.sample(),position,orientation,pose_stamp,now);}catch(const std::exception&){return stop("FAST-LIO and map pose projection invalid");}
    if(!braking_clear(target,now,projection.yaw,projection.position)||
       !braking_clear(projection.velocity,now,0,projection.position))return stop("Braking corridor obstructed");
    out.velocity=target;out.moving=true;return out;
  }
private:
  std::optional<Session> session_;uint64_t sequence_=0;double enabled_at_=0;
  static bool fresh(double now,double stamp,double timeout){return std::isfinite(now)&&std::isfinite(stamp)&&now-stamp>=-.05&&now-stamp<=timeout;}
  bool braking_clear(const U& candidate,double now,double heading,const U& origin) const {
    if(!std::isfinite(deceleration)||!std::isfinite(jerk)||deceleration<=0||jerk<=0)return false;
    double speed=candidate.norm(),horizon=std::max(.2,speed/deceleration+deceleration/jerk+.2);
    if(horizon>3||!std::isfinite(speed))return false;
    double c=std::cos(heading),s=std::sin(heading);U velocity(c*candidate.x()-s*candidate.y(),s*candidate.x()+c*candidate.y());
    double dt=std::min(.025,grid.resolution/(2*std::max(speed,.01)));
    auto aligned=obstacles;
    for(auto& obstacle:aligned)obstacle.age=std::max(0.0,now-prediction_stamp);
    try{
      for(double t=0;t<=horizon+dt*.5;t+=dt){
        U p=origin+t*velocity;if(grid.at(p)>=253||!grid.body_known(p,radius+speed*dt*.5))return false;
        for(const auto& obstacle:aligned){
          if(obstacle.distance(p,t).first<=radius)return false;
        }
      }
    }catch(const std::exception&){return false;}
    return true;
  }
};
}
