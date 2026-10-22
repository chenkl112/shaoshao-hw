#pragma once
#include "common.hpp"
#include <set>

namespace hw_runtime {
struct Metrics {bool converged=false;double error=INFINITY,overlap=0,min_eigenvalue=0,condition=INFINITY;};
inline bool quality_ok(const Metrics& m,const RegistrationSettings& p){return m.converged&&std::isfinite(m.error)&&std::isfinite(m.overlap)&&std::isfinite(m.min_eigenvalue)&&std::isfinite(m.condition)&&m.error>=0&&m.error<=p.max_error&&m.overlap>=p.min_overlap&&m.overlap<=1&&m.min_eigenvalue>=p.min_eigenvalue&&m.condition>=1&&m.condition<=p.max_condition;}
inline std::optional<std::pair<Pose,Metrics>> unique_best(std::vector<std::pair<Pose,Metrics>> values,const RegistrationSettings& p){
  std::erase_if(values,[&](const auto& v){return !quality_ok(v.second,p);});if(values.empty())return {};
  std::stable_sort(values.begin(),values.end(),[](const auto& a,const auto& b){return a.second.error<b.second.error;});
  for(size_t i=1;i<values.size();++i){auto [d,a]=delta(values[0].first,values[i].first);if((d>=.5||a>=.25)&&values[i].second.error<std::max(values[0].second.error*1.25,values[0].second.error+1e-6))return {};}
  return values[0];
}
class Policy {
public:
  RegistrationSettings settings;std::optional<Pose> correction,raw,pending,odom;
  std::optional<double> last_stamp,accepted_at,rejected_at,odom_at,scan_at;
  bool initialized=false,scan_valid=false;int confirmations=0;uint64_t accepted=0,rejected=0;
  double distance=0,accepted_distance=0,clearance=0;std::string reason="等待自动全局定位";Metrics metrics;
  explicit Policy(RegistrationSettings p={}):settings(p){}
  void odometry(const Pose& p,double stamp){
    if(!std::isfinite(stamp)||!p.matrix().allFinite())return;
    if(odom_at){if(stamp<=*odom_at)return;double d=delta(*odom,p).first;if(d>std::max(.15,1.5*(stamp-*odom_at)))lose("里程计跳变，正在重新全局定位");distance+=d;}
    odom=p;odom_at=stamp;
  }
  void scan(double stamp,const std::vector<Eigen::Vector2d>& points){
    if(!std::isfinite(stamp)||(scan_at&&stamp<=*scan_at))return;scan_at=stamp;std::set<int> sectors;clearance=INFINITY;
    for(const auto& p:points){if(!p.allFinite())continue;int sector=static_cast<int>((std::atan2(p.y(),p.x())+pi)/(pi/4))%8;sectors.insert(sector);clearance=std::min(clearance,p.norm());}
    scan_valid=points.size()>=60&&sectors.size()>=6;if(!std::isfinite(clearance))clearance=0;
  }
  void reject(const std::string& why,double now){++rejected;rejected_at=now;reason=why;confirmations=0;if(!initialized)pending.reset();}
  void lose(const std::string& why){initialized=false;pending.reset();confirmations=0;reason=why;}
  bool observe(const Pose& p,const Metrics& m,double stamp,double now,bool global=false){
    if(last_stamp&&stamp<=*last_stamp)return false;last_stamp=stamp;
    if(!std::isfinite(stamp)||now-stamp>.5||stamp>now+.05){reject("配准结果过期",now);return false;}
    metrics=m;if(!quality_ok(m,settings)||!p.matrix().allFinite()){reject("点云匹配质量不足",now);return false;}
    auto ref=initialized?raw:pending;
    if(ref){auto [d,a]=delta(*ref,p);if(d>(initialized?settings.max_step:.1)||a>(initialized?settings.max_angle:.1)){reject("校正跳变超限",now);return false;}}
    ++confirmations;
    if(!initialized){
      if(!global){reject("等待唯一全局候选",now);return false;}pending=p;
      if(confirmations<settings.confirmations){reason="正在确认全局定位（"+std::to_string(confirmations)+"/"+std::to_string(settings.confirmations)+"）";return false;}
      correction=p;initialized=true;
    }else correction=interpolate(*correction,p,settings.alpha);
    raw=p;accepted_at=stamp;accepted_distance=distance;++accepted;reason="地图定位就绪";return true;
  }
  Json::Value status(double now,double radius,double padding,double max_speed){
    Json::Value result;result["ready"]=false;result["state"]="acquiring";result["message"]=reason;result["speed_limit_mps"]=0.;
    result["last_correction_age_s"]=accepted_at?Json::Value(std::max(0.,now-*accepted_at)):Json::Value();
    double travelled=std::max(0.,distance-accepted_distance);result["distance_since_correction_m"]=travelled;
    result["accepted"]=Json::UInt64(accepted);result["rejected"]=Json::UInt64(rejected);result["confirmations"]=confirmations;
    result["metrics"]["converged"]=metrics.converged;
    for(const auto& [name,value]:std::vector<std::pair<std::string,double>>{{"error",metrics.error},{"overlap",metrics.overlap},{"min_eigenvalue",metrics.min_eigenvalue},{"condition",metrics.condition}})result["metrics"][name]=std::isfinite(value)?Json::Value(value):Json::Value();
    if(!initialized)return result;
    std::string why;if(!odom_at||now-*odom_at>1)why="里程计更新超时";
    if((!scan_at||now-*scan_at>settings.scan_timeout||!scan_valid)&&why.empty())why="扫描超时或回波覆盖不足";
    double age=now-*accepted_at;if(age>settings.timeout&&why.empty())why="有效地图校正超时";
    if(!why.empty()){lose(why);result["state"]="lost";result["message"]=why;return result;}
    bool degraded=age>settings.fresh||(rejected_at&&*rejected_at>*accepted_at)||confirmations<settings.confirmations;
    if(degraded){
      if(travelled>settings.hold_distance){lose("退化累计移动超限");result["state"]="lost";result["message"]=reason;return result;}
      double required=radius+padding+settings.hold_distance+travelled+max_speed*std::max(0.,now-*scan_at);
      result["state"]="degraded";result["required_clearance_m"]=required;result["observed_clearance_m"]=clearance;
      if(clearance<=required){result["message"]="定位退化且近场余量不足，已停车";return result;}
      result["ready"]=true;result["message"]="定位退化，低速衔接";result["speed_limit_mps"]=std::min(max_speed,settings.degraded_speed);return result;
    }
    result["ready"]=true;result["state"]="tracking";result["message"]="地图定位就绪";result["speed_limit_mps"]=max_speed;return result;
  }
};
}
