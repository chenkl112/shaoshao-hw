#include "hw_omni_controller/planner_defaults.hpp"
#include "hw_omni_controller/trajectory_bridge.hpp"
#include "hw_omni_controller/recovery.hpp"
#include <iostream>
#include "hw_omni_controller/navigation_grid.hpp"
using namespace hw_omni;
void require(bool v,const char* text){if(!v)throw std::runtime_error(text);}
auto clock_at(double t){return std::chrono::steady_clock::time_point(std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(t)));}
struct Fixture {
  nav_executor::GridGeometry geometry{160,160,.05,U(-4,-4)};
  std::vector<uint8_t> cells=std::vector<uint8_t>(25600,0);
  nav_executor::DirectionMap::ConstPtr flat=std::make_shared<nav_executor::DirectionMap>(geometry,std::vector<U>(25600,U::Zero()),std::vector<uint8_t>(25600,0));
  auto cost()const{return std::make_shared<nav_executor::CostMap>(geometry,cells);}
  void box(double x0,double y0,double x1,double y1,uint8_t value){for(int y=0;y<160;++y)for(int x=0;x<160;++x){U p=geometry.cell_center({x,y});if(p.x()>=x0&&p.x()<=x1&&p.y()>=y0&&p.y()<=y1)cells[y*160+x]=value;}}
};
nav_executor::PlannerConfig config(){nav_executor::CapabilityProfile p{{{0,.45},{-20,20}},{.3,50,.3}};auto c=planner_defaults(p,{p,p,p});c.endpoint_goal_reached_distance=0;c.endpoint_nudge_max_distance=0;c.kino_a_star.dynamics.velocity_max=.5;c.kino_a_star.dynamics.tangential_acceleration_max=.3;c.kino_a_star.dynamics.lateral_acceleration_max=.3;c.kino_a_star.start_yaw_relaxation={16,0,0,.5};c.occupied_cost_threshold=253;return c;}
nav_executor::AnnotatedPath::ConstPtr planning(){
  Fixture f;auto mask=std::make_shared<nav_executor::RouteTerrainMask>(nav_executor::RouteTerrainMaskParams{.6,.6,1});
  auto logger=rclcpp::get_logger("hw_navigation_test");nav_executor::PathPlanner planner(config(),mask,logger);
  nav_executor::PlanRequest r;r.goal={1,U(2,0),false};r.current_pos_map=U(0,0);r.direction_map=f.flat;
  auto run=[&]{auto cost=f.cost();mask->initialize(*cost,f.flat);r.obstacles.global_static=cost;r.obstacles.hard_cost=cost;return planner.plan_snapshot(r);};
  auto open=run();if(open.kind!=nav_executor::PlanResult::Kind::PATH)std::cerr<<open.failure_reason<<'\n';require(bool(open.path),"Full HW pipeline must solve open space");
  for(double distance:{.031,.06,.12,.2,.49}){
    r.goal.position_map=U(distance,0);auto short_plan=run();if(!short_plan.path)std::cerr<<distance<<": "<<short_plan.failure_reason<<'\n';require(bool(short_plan.path),"Centimeter-scale goal must have a valid accelerating/stopping profile");require(short_plan.path->speed_profile.total_time()>0,"Short route must have finite positive traversal time");
  }
  r.goal.position_map=U(2,0);
  f.box(.8,-.4,1.2,.4,254);auto detour=run();if(!detour.path)std::cerr<<detour.failure_reason<<'\n';require(bool(detour.path),"HW pipeline must route around a blocking wall");
  double max_y=0;for(double s=0;s<detour.path->trajectory.total_arc_length();s+=.01){auto p=detour.path->trajectory.eval_arc_length(s).p;require(f.cost()->sample_map(p)->value<253,"MINCO must not cut through wall");max_y=std::max(max_y,std::abs(p.y()));}
  require(max_y>.4,"HW pipeline must produce a real detour");
  require(detour.path->speed_profile.states().back().velocity<1e-8,"HW speed profile must stop at goal");
  for(auto state:detour.path->speed_profile.states())require(state.velocity<=.450001,"HW speed profile must respect omni limits");
  f.box(.8,-4,1.2,4,254);require(!run().path,"Disconnected map must report failure");
  f.box(-4,-4,4,4,0);f.box(1.9,-.1,2.1,.1,255);require(!run().path,"Unknown goal must be rejected");
  f.box(-4,-4,4,4,0);int checks=0;planner.set_cancel_checker([&]{return ++checks>4;});bool canceled=false;try{run();}catch(const std::exception&){canceled=true;}require(canceled,"Planning must respond to in-search cancellation");
  nav_msgs::msg::Path path;path.header.frame_id="map";path.header.stamp.sec=1;
  for(int i=0;i<=100;++i){auto sample=open.path->trajectory.eval_arc_length(open.path->trajectory.total_arc_length()*i/100);geometry_msgs::msg::PoseStamped p;p.pose.position.x=sample.p.x();p.pose.position.y=sample.p.y();p.pose.orientation.w=1;path.poses.push_back(p);}
  auto msg=encode_trajectory(*open.path,path);auto decoded=decode_trajectory(msg);require(std::abs(decoded->trajectory.total_arc_length()-open.path->trajectory.total_arc_length())<1e-8,"MINCO bridge must preserve exact curve");
  auto stale=path;stale.header.stamp.sec=2;require(!same_plan(stale,path),"Stale path identity must not match");
  msg.profile_speed[1]=1;bool rejected=false;try{decode_trajectory(msg);}catch(const std::exception&){rejected=true;}require(rejected,"Invalid profile must be rejected");return open.path;
}
void recoveries(nav_executor::AnnotatedPath::ConstPtr path){
  Fixture f;auto cost=f.cost();Recovery recovery(rclcpp::get_logger("hw_recovery_test"));nav_executor::RouteEstimate route;route.path=path;route.status=nav_executor::RouteTrackingStatus::TRACKED;route.reference_position=U(.2,0);
  RecoveryDecision out;for(int i=0;i<=125;++i)out=recovery.update(U::Zero(),U::Zero(),U::Zero(),&route,*cost,*f.flat,clock_at(i*.05));
  require(out.state==nav_executor::MotionState::STUCK_REVERSE,"Six seconds without arc progress must trigger reverse");require(out.target&&out.target->x()<-.3,"Reverse must oppose translation direction");
  out=recovery.update(U(-.36,0),U::Zero(),U::Zero(),&route,*cost,*f.flat,clock_at(6.4));require(out.replan,"Reverse completion must request original-goal replanning");
  recovery.reset();for(int i=0;i<=42;++i)out=recovery.update(U::Zero(),U::Zero(),U(.45,0),&route,*cost,*f.flat,clock_at(i*.05));require(out.state==nav_executor::MotionState::STUCK_REVERSE,"Two seconds of commanded motion without displacement must trigger reverse");
  out=recovery.update(U::Zero(),U::Zero(),U::Zero(),&route,*cost,*f.flat,clock_at(6));require(out.replan,"Reverse timeout must terminate the recovery attempt");
  recovery.reset();f.box(-.3,-.3,.3,.3,100);cost=f.cost();out=recovery.update(U::Zero(),U::Zero(),U::Zero(),&route,*cost,*f.flat,clock_at(0));require(out.state==nav_executor::MotionState::HAZARD_RECOVERY&&out.target,"Hazard must trigger HW polar safe-point search");
  auto target=*out.target;require(cost->sample_map(target)->value<5,"Safe point must be outside inflated hazard");
  for(int i=1;i<=24;++i){out=recovery.update(target,U::Zero(),U::Zero(),&route,*cost,*f.flat,clock_at(i*.05));if(out.replan)break;}require(out.replan,"One second in safe space must request replanning");
  recovery.reset();require(!recovery.reject_rollout()&&!recovery.reject_rollout()&&recovery.reject_rollout(),"Exactly three lethal rejections must request replanning");recovery.accept_rollout();require(!recovery.reject_rollout(),"Safe rollout must reset consecutive rejection count");
  f.box(-4,-4,4,4,0);f.box(-.15,-.15,.15,.15,100);
  f.box(-.25,-.25,-.2,.25,254);f.box(.2,-.25,.25,.25,254);f.box(-.25,-.25,.25,-.2,254);f.box(-.25,.2,.25,.25,254);
  cost=f.cost();recovery.reset();out=recovery.update(U::Zero(),U::Zero(),U::Zero(),&route,*cost,*f.flat,clock_at(0));require(!out.target,"Safe endpoints behind a lethal barrier must not be escape candidates");
  f.box(-4,-4,4,4,0);f.box(-.15,-.15,.15,.15,100);cost=f.cost();out=recovery.update(U::Zero(),U::Zero(),U::Zero(),&route,*cost,*f.flat,clock_at(1));require(!out.target,"Failed recovery search must respect cooldown");
  out=recovery.update(U::Zero(),U::Zero(),U::Zero(),&route,*cost,*f.flat,clock_at(4.1));require(bool(out.target),"Safe-point search must retry after the 4-second cooldown");
  f.box(-4,-4,4,4,254);cost=f.cost();recovery.reset();out=recovery.update(U::Zero(),U::Zero(),U::Zero(),&route,*cost,*f.flat,clock_at(0));require(!out.target,"No safe escape must result in a stationary wait");
}
void circular_unknown(){
  Grid grid;grid.width=160;grid.height=160;grid.resolution=.025;grid.origin=U(-2,-2);grid.cells.assign(25600,0);
  grid.cells[92*160+92]=255; // diagonal square outside the circular body
  auto cost=recovery_cost_cells(grid,.317658);require(cost[80*160+80]==0,"Circular recovery field must not trap the vehicle beside diagonal unknown cells");
  grid.cells[80*160+88]=255;cost=recovery_cost_cells(grid,.317658);require(cost[80*160+80]==255,"Unknown cell inside full body must block recovery field");require(cost[2*160+2]==255,"Recovery field must include full body at map boundary");
}
int main(){try{circular_unknown();auto path=planning();recoveries(path);std::cout<<"HW pipeline, detour, disconnected/unknown goals, cancellation, trajectory binding, reverse and hazard recovery passed\n";return 0;}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
