#pragma once
#include "maps.hpp"
#include "configuration.hpp"
#include <functional>
#include <mutex>

namespace hw_runtime {
struct TaskHooks {
  std::function<void(const std::string&)> mode;
  std::function<void(uint64_t,const Json::Value&)> plan,goal;
  std::function<void()> cancel,retry;
};
class Tasks {
public:
  mutable std::recursive_mutex mutex;TaskHooks hooks;fs::path maps;
  std::string mode="idle",target_mode="idle",error,nav_status="idle",nav_message="请在地图上选择目标位置";
  bool switching=false,mapping_saved=false,localized=false,nav_server=false,stack_ready=false,serial_ready=false,hardware_ready=false,hardware_seen=false;
  double cloud_at=0,odom_at=0,pose_at=0,serial_at=0,goal_started=0,plan_started=0;uint64_t goal_generation=0,plan_generation=0,hardware_epoch=0;
  Json::Value pose,local_pose,localization,goal,path=Json::Value(Json::arrayValue),plan,remaining,motion,costmap;
  std::optional<Json::Value> resume;std::optional<uint64_t> resume_plan,resume_goal_generation;double resume_retry_at=0;
  bool goal_pending=false;uint64_t retry_count=0;double cancel_deadline=0;
  static constexpr double retry_period=.5;
  explicit Tasks(fs::path directory):maps(std::move(directory)){fs::create_directories(maps);clear_plan();costmap["available"]=false;}
  std::string reason(double now) const {
    if(switching)return "正在切换运行模式";if(mode!="navigation")return "请先点击顶部“导航”";if(!error.empty())return error;
    if(!localized)return localization.get("message","等待自动全局定位").asString();if(!hardware_ready)return "等待有效的底盘实际速度反馈与拨杆授权";
    if(!nav_server||!stack_ready)return "等待 Nav2 导航服务就绪";if(!pose.isObject()||now-pose_at>1)return "地图位置更新超时";
    if(!cloud_at||!odom_at||now-cloud_at>1||now-odom_at>1)return "点云或雷达里程计已超时";
    if(!serial_ready||now-serial_at>1)return "等待下位机通讯，请确认8081控制服务已运行";return {};
  }
  bool active() const{return nav_status=="sending"||nav_status=="active"||nav_status=="canceling";}
  bool enabled(double now) const{return nav_status=="active"&&reason(now).empty();}
  void clear_plan(){++plan_generation;plan=Json::Value(Json::objectValue);plan["status"]="idle";plan["message"]="选点后自动规划路径";plan["goal"]=Json::Value();plan["path"]=Json::Value(Json::arrayValue);path=Json::Value(Json::arrayValue);}
  void clear_resume(){resume.reset();resume_plan.reset();resume_goal_generation.reset();}
  void wait_and_retry(const std::string& message,double now){
    if(!goal_pending)return;
    resume=goal;resume_plan.reset();resume_goal_generation.reset();resume_retry_at=now+retry_period;
    nav_status="waiting";nav_message=message+"，停车等待后重新规划";remaining=Json::Value();++retry_count;
  }
  void pause(const std::string& message,double now=steady()){
    if(!goal_pending)return;
    resume=goal;if(resume_plan){++plan_generation;resume_plan.reset();}resume_goal_generation.reset();
    if(active())cancel(true,now);else nav_status="waiting";
    nav_message=message+"，保留目标并停车等待";
  }
  Json::Value navigation_target(const Json::Value& body) const {
    double x=finite_number(body,"x"),y=finite_number(body,"y");auto map=saved_map(maps).info;
    if(!map["available"].asBool())throw std::runtime_error(map["message"].asString());
    if(!body["map_revision"].isString()||body["map_revision"]!=map["revision"])throw std::runtime_error("地图已更新，请刷新地图后重新选点");
    double a=map["origin"][2].asDouble(),dx=x-map["origin"][0].asDouble(),dy=y-map["origin"][1].asDouble();double mx=std::cos(a)*dx+std::sin(a)*dy,my=-std::sin(a)*dx+std::cos(a)*dy;
    if(mx<0||my<0||mx>=map["width"].asDouble()*map["resolution"].asDouble()||my>=map["height"].asDouble()*map["resolution"].asDouble())throw std::invalid_argument("请选择地图范围内的目标");
    Json::Value out;out["x"]=x;out["y"]=y;out["map_revision"]=body["map_revision"];out["frame"]="map";return out;
  }
  void request_mode(const std::string& requested){
    if(requested!="idle"&&requested!="mapping"&&requested!="navigation")throw std::invalid_argument("Unknown mode");
    if(switching){if(target_mode==requested)return;throw std::runtime_error("正在切换模式，请稍候");}
    if(mode==requested)return;if(!hooks.mode)throw std::runtime_error("Native mode manager unavailable");
    cancel(false);clear_resume();clear_plan();target_mode=requested;switching=true;error.clear();
    try{hooks.mode(requested);}catch(...){switching=false;target_mode=mode;throw;}
  }
  void mode_finished(const std::string& actual,const fs::path& directory,const std::string& failure={}){
    std::lock_guard<std::recursive_mutex> lock(mutex);mode=actual;maps=directory;switching=false;error=failure;target_mode=actual;
    localized=false;stack_ready=false;nav_server=false;pose=Json::Value();pose_at=cloud_at=odom_at=0;clear_plan();
    ++goal_generation;goal_pending=false;retry_count=0;nav_status="idle";nav_message="请在地图上选择目标位置";goal=Json::Value();remaining=Json::Value();clear_resume();
  }
  void request_plan(const Json::Value& body,double now,bool automatic=false){
    auto target=navigation_target(body);auto why=reason(now);if(!why.empty())throw std::runtime_error(why);if(active()||(!automatic&&goal_pending))throw std::runtime_error("已有导航任务，请先取消或等待完成");if(!hooks.plan)throw std::runtime_error("路径规划服务不可用");
    ++plan_generation;plan_started=now;plan["status"]="planning";plan["message"]="正在计算路径…";plan["goal"]=target;plan["path"]=Json::Value(Json::arrayValue);
    try{hooks.plan(plan_generation,target);}catch(const std::exception& e){plan["status"]="failed";plan["message"]=e.what();throw;}
  }
  void plan_result(uint64_t generation,const std::vector<Eigen::Vector2d>& points,const std::string& failure={}){
    std::lock_guard<std::recursive_mutex> lock(mutex);if(generation!=plan_generation)return;
    auto map=saved_map(maps).info;
    if(!failure.empty()||points.empty()||plan["goal"]["map_revision"]!=map["revision"]){plan["status"]="failed";plan["message"]=failure.empty()?"地图已变化或规划路径为空":failure;plan["path"]=Json::Value(Json::arrayValue);return;}
    double length=0;for(size_t i=0;i<points.size();++i){if(!points[i].allFinite()){plan_result(generation,{},"规划路径包含无效值");return;}if(i)length+=(points[i]-points[i-1]).norm();}
    if((points.back()-Eigen::Vector2d(plan["goal"]["x"].asDouble(),plan["goal"]["y"].asDouble())).norm()>.3){plan_result(generation,{},"规划路径终点不匹配");return;}
    size_t step=std::max(size_t(1),(points.size()+1999)/2000);Json::Value selected(Json::arrayValue);
    auto add=[&](const auto& p){Json::Value point(Json::arrayValue);point.append(p.x());point.append(p.y());selected.append(point);};for(size_t i=0;i<points.size();i+=step)add(points[i]);if((points.size()-1)%step)add(points.back());
    plan["status"]="ready";plan["message"]="路径已计算";plan["path"]=selected;plan["length_m"]=length;
  }
  void request_goal(const Json::Value& body,double now,bool automatic=false){
    auto target=navigation_target(body);auto why=reason(now);if(!why.empty())throw std::runtime_error(why);if(active()||(!automatic&&goal_pending))throw std::runtime_error("已有导航任务，请先取消或等待完成");if(!hooks.goal)throw std::runtime_error("导航服务不可用");
    if(!automatic){clear_resume();retry_count=0;}goal_pending=true;
    ++goal_generation;goal=target;goal["yaw"]=pose.get("yaw",0.);goal_started=now;++plan_generation;path=Json::Value(Json::arrayValue);nav_status="sending";nav_message="正在发送导航目标…";remaining=Json::Value();
    try{hooks.goal(goal_generation,goal);}catch(const std::exception& e){wait_and_retry(e.what(),now);}
  }
  void goal_response(uint64_t generation,bool accepted,double now=-1){
    std::lock_guard<std::recursive_mutex> lock(mutex);if(generation!=goal_generation)return;
    if(now<0)now=goal_started;
    if(!accepted){if(goal_pending)wait_and_retry("导航目标暂时被拒绝",now);else{nav_status="canceled";nav_message="导航已取消";}return;}
    if(!goal_pending||nav_status=="canceling"||switching||mode!="navigation"){cancel_safely();return;}
    nav_status="active";nav_message="正在前往目标";if(resume_goal_generation==generation)clear_resume();
  }
  void goal_result(uint64_t generation,const std::string& status,const std::string& message,double now=-1){
    std::lock_guard<std::recursive_mutex> lock(mutex);if(generation!=goal_generation)return;if(now<0)now=goal_started;
    if(status=="succeeded"){goal_pending=false;clear_resume();nav_status=status;nav_message=message;}
    else if(goal_pending&&(status!="canceled"||resume)){wait_and_retry(message,now);}
    else{goal_pending=false;clear_resume();nav_status="canceled";nav_message="导航已取消";}
    remaining=Json::Value();
  }
  void cancel(bool preserve_resume=false,double now=steady()){
    if(!preserve_resume){goal_pending=false;clear_resume();}
    if(!active()){if(!preserve_resume&&nav_status=="waiting"){nav_status="canceled";nav_message="导航已取消";++goal_generation;}return;}
    if(nav_status=="canceling")return;
    nav_status="canceling";nav_message="正在取消导航…";cancel_deadline=now+3;cancel_safely();
  }
  void cancel_safely() noexcept {try{if(hooks.cancel)hooks.cancel();}catch(const std::exception& e){nav_message=std::string("输出已禁用，取消请求失败：")+e.what();}catch(...){nav_message="输出已禁用，取消请求失败";}}
  void hardware(bool ready,uint64_t epoch,bool revoked=false){
    std::lock_guard<std::recursive_mutex> lock(mutex);bool changed=hardware_seen&&hardware_epoch!=epoch;hardware_seen=true;hardware_ready=ready;hardware_epoch=epoch;
    if(revoked||changed){cancel();clear_resume();}
    else if(!ready)pause("实际速度反馈不可用");
  }
  void localization_state(const Json::Value& value){
    std::lock_guard<std::recursive_mutex> lock(mutex);localization=value;localized=value.get("ready",false).asBool();auto state=value.get("state","acquiring").asString();
    if(!localized&&goal_pending)pause("定位尚未就绪");
  }
  void tick(double now){
    std::lock_guard<std::recursive_mutex> lock(mutex);
    bool map_ready=true;
    if(goal_pending){
      auto map=saved_map(maps).info;
      if(!map["available"].asBool()){pause("地图暂不可用",now);map_ready=false;}
      else if(map["revision"]!=goal["map_revision"]){cancel(false,now);error="地图已变化，请退出导航并重新进入";nav_message=error;}
    }
    if(nav_status=="sending"&&now-goal_started>10)pause("导航响应超时",now);
    if(nav_status=="canceling"&&now>=cancel_deadline){
      cancel_safely();++goal_generation;
      if(goal_pending)wait_and_retry("导航取消响应超时，输出已禁用",now);else{nav_status="canceled";nav_message="导航已取消，输出已禁用";}
    }
    if(plan["status"].asString()=="planning"&&now-plan_started>10){++plan_generation;plan["status"]="failed";plan["message"]="路径规划超时";}
    if(goal_pending&&map_ready&&resume&&!active()&&reason(now).empty()&&localization["state"].asString()=="tracking"&&now>=resume_retry_at){
      try{
        auto map=saved_map(maps).info;if((*resume)["map_revision"]!=map["revision"]){cancel(false,now);return;}
        if(!resume_plan){request_plan(*resume,now,true);resume_plan=plan_generation;}
        else if(*resume_plan==plan_generation&&plan["status"].asString()=="ready"){auto target=*resume;resume_goal_generation=goal_generation+1;request_goal(target,now,true);}
        else if(plan["status"].asString()=="failed"){wait_and_retry("原目标路径暂不可通行",now);}
      }catch(const std::exception& e){wait_and_retry(std::string("自动续行暂不可用：")+e.what(),now);}
    }
  }
  Json::Value state(double now) const {
    std::lock_guard<std::recursive_mutex> lock(mutex);auto why=reason(now);Json::Value s;
    s["mode"]=mode;s["switching"]=switching;s["error"]=error;s["demo"]=false;s["preview"]=mode!="navigation";s["mapping_only"]=true;s["navigation_available"]=true;s["pose"]=now-odom_at<=1?local_pose:Json::Value();s["cloud_age"]=cloud_at?Json::Value(std::max(0.,now-cloud_at)):Json::Value();s["odom_age"]=odom_at?Json::Value(std::max(0.,now-odom_at)):Json::Value();
    s["yaw"]["ready"]=false;s["yaw"]["reason"]="雷达坐标平移，小陀螺独立控制";s["spin"]["enabled"]=false;s["spin"]["rate_rad_s"]=.5;s["spin"]["max_rate_rad_s"]=.5;
    s["point_topic"]=mode=="idle"?"/livox/lidar":"/cloud_registered";
    auto& c=s["mapping_control"];c["running"]=mode=="mapping";c["starting"]=switching&&target_mode=="mapping";c["stopping"]=switching&&mode=="mapping";c["can_start"]=!switching&&mode!="mapping";c["can_stop"]=!switching&&mode=="mapping";c["maps_dir"]=maps.string();c["saved"]=mapping_saved;c["error"]=error;c["message"]=switching?"正在切换并保存地图…":mode=="mapping"?"正在建图，可停止并保存":"待机，可开始建图";
    auto filter=mapping_filter(maps);filter["locked"]=switching||mode!="idle";filter["reference"]="开始建图时的雷达高度";s["mapping_filter"]=filter;
    auto& n=s["navigation"];n["ready"]=why.empty();n["reason"]=why;n["status"]=nav_status;n["message"]=nav_message;n["localized"]=localized&&pose.isObject()&&now-pose_at<=1;n["localization"]=localization;n["goal"]=goal;n["plan"]=plan;n["path"]=path;n["distance_remaining"]=remaining;n["pose"]=pose.isObject()&&now-pose_at<=1?pose:Json::Value();n["motion"]=motion;n["auto_resume_pending"]=bool(resume);n["goal_pending"]=goal_pending;n["retry_count"]=Json::UInt64(retry_count);return s;
  }
  Json::Value dispatch(const Json::Value& request,double now){
    std::lock_guard<std::recursive_mutex> lock(mutex);auto operation=request["operation"].asString();
    if(operation=="snapshot"){auto map=saved_map(maps);Json::Value out;out["state"]=state(now);out["map"]=map.info;out["image_path"]=map.image.string();return out;}
    if(operation=="costmap")return costmap;if(operation!="command"||!request["body"].isObject())throw std::invalid_argument("Unsupported supervisor request");
    auto route=request["path"].asString();const auto& body=request["body"];
    if(route=="/api/mode")request_mode(body["mode"].asString());
    else if(route=="/api/navigation/plan")request_plan(body,now);
    else if(route=="/api/navigation/goal")request_goal(body,now);
    else if(route=="/api/navigation/cancel")cancel();
    else if(route=="/api/localization/retry"){if(mode!="navigation"||!hooks.retry)throw std::runtime_error("定位服务未运行");if(active()){resume=goal;cancel(true);}localized=false;hooks.retry();}
    else if(route=="/api/mapping/filter"){
      if(mode!="idle"||switching)throw std::runtime_error("请先进入待机，再修改建图过滤设置");
      if(!body["enabled"].isBool()||!body["dynamic_enabled"].isBool()||finite_number(body,"height_m")<=0||body["height_m"].asDouble()>50)throw std::invalid_argument("Invalid mapping filter");write_json(maps/"mapping_filter.json",body);
    }else if(route=="/api/spin"||route=="/api/spin/heartbeat"){
      if(body.get("enabled",false).asBool())throw std::runtime_error("请从独立手动入口控制小陀螺");
    }else throw std::invalid_argument("Unsupported task route");return state(now);
  }
};
}
