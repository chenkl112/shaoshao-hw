#include "hw_omni_controller/mpc.hpp"
#include <chrono>
#include <iostream>
#include <stdexcept>
using namespace hw_omni;
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
Problem reference(U goal){
  Problem p;p.settings.iterations=12;
  for(int i=0;i<=N;++i){p.positions[i]=goal;p.velocities[i].setZero();p.lidar_yaws[i]=0;}
  return p;
}
void dynamics_and_cost_derivatives(){
  auto p=reference(U(0.2,0.4));p.settings.tau_y=0.45;p.lidar_yaws.fill(0.7);
  X x; x<<0.1,-0.2,0.2,0.1,0.18,0.12,0.001,-0.001;
  U u(0.181,0.119);XX fx;XU fu;p.dynamics_jacobians(0,x,u,fx,fu);
  const double e=1e-6;
  for(int i=0;i<NX;++i){X plus=x,minus=x;plus[i]+=e;minus[i]-=e;require(((p.dynamics(0,plus,u)-p.dynamics(0,minus,u))/(2*e)-fx.col(i)).norm()<1e-7,"Dynamics state Jacobian");}
  for(int i=0;i<NU;++i){U plus=u,minus=u;plus[i]+=e;minus[i]-=e;require(((p.dynamics(0,x,plus)-p.dynamics(0,x,minus))/(2*e)-fu.col(i)).norm()<1e-7,"Dynamics control Jacobian");}
  X lx;U lu;XX lxx;UX lux;Eigen::Matrix2d luu;p.running_cost_derivatives(0,x,u,lx,lu,lxx,lux,luu);
  for(int i=0;i<NX;++i){X plus=x,minus=x;plus[i]+=e;minus[i]-=e;require(std::abs((p.running_cost(0,plus,u)-p.running_cost(0,minus,u))/(2*e)-lx[i])<1e-6,"Cost state gradient");}
  for(int i=0;i<NU;++i){U plus=u,minus=u;plus[i]+=e;minus[i]-=e;require(std::abs((p.running_cost(0,x,plus)-p.running_cost(0,x,minus))/(2*e)-lu[i])<1e-6,"Cost control gradient");}
}
void closed_loop(U goal){
  X x=X::Zero();U previous=U::Zero(),delta=U::Zero();
  double total_ms=0;
  for(int i=0;i<120;++i){
    auto p=reference(goal);auto start=std::chrono::steady_clock::now();auto result=solve(p,x);
    total_ms+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
    require(result.valid,"Closed-loop MPC returned invalid rollout");
    U change=result.command-previous;
    require(result.command.norm()<=p.settings.speed+1e-7,"Vector speed bound");
    require(change.norm()/DT<=p.settings.acceleration+1e-7,"Vector target acceleration bound");
    require((change-delta).norm()/(DT*DT)<=p.settings.jerk+1e-7,"Vector target jerk bound");
    x=p.dynamics(0,x,result.command);previous=result.command;delta=change;
  }
  std::cout<<"goal "<<goal.transpose()<<" final "<<x.head<2>().transpose()<<" mean_solve_ms "<<total_ms/120<<'\n';
  require((x.head<2>()-goal).norm()<0.04,"Closed-loop position tracking");
  require(x.segment<2>(2).norm()<0.04,"Closed-loop stopping");
}
void predictions_and_collisions(){
  Obstacle obstacle;obstacle.dt=0.1;obstacle.age=0.13;
  obstacle.footprint={U(-.1,-.1),U(.1,-.1),U(.1,.1),U(-.1,.1)};
  for(int i=0;i<=30;++i)obstacle.centers.emplace_back(i*.1,1.0);
  require((obstacle.center(.05)-U(.18,1)).norm()<1e-8,"Timestamp-aligned interpolation");
  require(obstacle.distance(U(.13,1),0).first<0,"Inside footprint signed distance");
  bool expired=false;try{obstacle.center(4);}catch(const std::out_of_range&){expired=true;}
  require(expired,"Expired prediction must not be clamped");
  auto p=reference(U(0,.5));
  // A stationary robot must also reject an obstacle that crosses it later.
  obstacle.age=0;obstacle.centers.clear();
  for(int i=0;i<=30;++i)obstacle.centers.emplace_back(1-i*.1,0);
  p.obstacles={obstacle};
  std::array<X,N+1> xs;for(auto& v:xs)v.setZero();std::array<U,N> us;for(auto& v:us)v.setZero();
  require(!p.safe(xs,us),"Future crossing of stationary robot");
  p.obstacles.clear();Grid grid;grid.width=80;grid.height=80;grid.resolution=.025;grid.origin=U(-1,-1);grid.cells.assign(6400,0);grid.cells[40*80+40]=254;p.grid=&grid;
  require(!p.safe(xs,us),"Static lethal obstacle rejection");
  grid.cells[40*80+40]=255;require(!p.safe(xs,us),"Unknown-cell rejection");
}
void predictive_avoidance(){
  auto p=reference(U(.7,0));p.settings.iterations=20;
  X initial=X::Zero();auto unpredicted=solve(p,initial);
  Obstacle obstacle;obstacle.footprint={U(-.1,-.1),U(.1,-.1),U(.1,.1),U(-.1,.1)};
  for(int i=0;i<=30;++i)obstacle.centers.emplace_back(.45,.8-i*.1);
  p.obstacles={obstacle};
  require(!p.safe(unpredicted.states,unpredicted.commands),"Crossing obstacle must invalidate the original rollout");
  auto predicted=solve(p,initial);
  require(predicted.valid,"Predictive avoidance must find a feasible rollout in the crossing fixture");
  require(p.safe(predicted.states,predicted.commands),"Optimized rollout must avoid the moving footprint");
}
void full_rollout_speed(){
  // An oblique path can push future controls outside the speed disk even
  // though every QP axis stays within [-speed,speed]. Check every output and
  // re-rolled future step with a response faster than the initial model.
  X x=X::Zero();
  for(int i=0;i<100;++i){
    auto p=reference(U(1.,.4));auto result=solve(p,x);
    require(result.valid,"Oblique path must retain a feasible complete rollout");
    require(p.safe(result.states,result.commands),"Backtracked rollout must satisfy collisions and vector speed");
    for(int k=0;k<N;++k){auto bounds=p.control_bounds(k,result.states[k]);
      require((result.commands[k].array()>=bounds.lower.array()-1e-7).all()&&
        (result.commands[k].array()<=bounds.upper.array()+1e-7).all(),"Backtracking preserves target acceleration and jerk bounds");}
    auto next=p.dynamics(0,x,result.command);x=next;
    // Synthetic plant mismatch, not a change to commissioned response params.
    x.segment<2>(2)+=.15*(result.command-x.segment<2>(2));
  }
  require((x.head<2>()-U(1.,.4)).norm()<.08,"Constrained oblique path must make progress");
}
void unknown_body_and_speed_transition(){
  Grid grid;grid.width=160;grid.height=160;grid.resolution=.025;grid.origin=U(-2,-2);grid.cells.assign(25600,255);
  for(int y=76;y<=83;++y)for(int x=0;x<160;++x)grid.cells[y*160+x]=0;
  for(int y=64;y<96;++y)for(int x=64;x<96;++x)grid.cells[y*160+x]=0;
  auto p=reference(U(.5,0));p.grid=&grid;
  require(grid.body_known(U::Zero(),p.settings.radius),"Initial whole body is known");
  require(!grid.body_known(U(.3,0),p.settings.radius),"Known center is not sufficient when body overlaps unknown");
  auto result=solve(p,X::Zero());
  if(result.valid)for(const auto& state:result.states)require(grid.body_known(state.head<2>(),p.settings.radius),"Every accepted rollout must keep the entire body known");
  require(!grid.body_known(U(1.9,0),p.settings.radius),"Body crossing costmap boundary must be blocked");
  require(speed_limit_hold(.05,U(.3,0),U(.3,0)),"Reduced speed cap starts safety braking hold");
  require(speed_limit_hold(.05,U::Zero(),U(.1,0)),"Hold waits for actual speed, not only the zero target");
  require(!speed_limit_hold(.05,U::Zero(),U(.04,0)),"Settled vehicle can re-enter MPC under reduced limit");
  auto slow=reference(U(.5,0));slow.settings.speed=.05;X initial=X::Zero();initial[2]=.04;
  require(solve(slow,initial).valid,"Zero command history after hold permits feasible degraded-speed MPC");
}
int main(){
  try{dynamics_and_cost_derivatives();closed_loop(U(.5,0));closed_loop(U(0,.5));closed_loop(U(.35,.35));predictions_and_collisions();predictive_avoidance();full_rollout_speed();unknown_body_and_speed_transition();std::cout<<"All native MPC tests passed\n";}
  catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
