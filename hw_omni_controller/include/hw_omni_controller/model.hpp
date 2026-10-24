#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace hw_omni {
constexpr int N = 40;
constexpr int NX = 8;
constexpr int NU = 2;
constexpr double DT = 0.05;
using X = Eigen::Matrix<double, NX, 1>;
using U = Eigen::Vector2d;
using XX = Eigen::Matrix<double, NX, NX>;
using XU = Eigen::Matrix<double, NX, NU>;
using UX = Eigen::Matrix<double, NU, NX>;
// A reduced hard speed bound cannot be applied to the old command history
// while also preserving its acceleration/jerk bounds. Enter an explicit zero
// hold until actual translation settles, then restart MPC from zero targets.
inline bool speed_limit_hold(double limit,const U& previous,const U& measured){
  return previous.norm()>limit+1e-7||measured.norm()>limit+.005;
}

struct Settings {
  // Initial whole-vehicle response assumptions; identify from measured logs.
  double tau_x = 0.30, tau_y = 0.30;
  double speed = 0.5, acceleration = 0.3, jerk = 1.0;
  double radius = 0.317658, obstacle_margin = 0.15;
  double position_weight = 12.0, velocity_weight = 2.0;
  double command_weight = 0.02, acceleration_weight = 0.2;
  double jerk_weight = 0.02, obstacle_weight = 1200.0, static_cost_weight = 8.0;
  int iterations = 20;
  void validate() const {
    for (double value : {tau_x, tau_y, speed, acceleration, jerk, radius,
         obstacle_margin, position_weight, velocity_weight, command_weight,
         acceleration_weight, jerk_weight, obstacle_weight, static_cost_weight})
      if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("Invalid MPC setting");
    if (speed > 0.5 || iterations < 1 || iterations > 100)
      throw std::invalid_argument("Invalid MPC speed/iterations");
  }
};

struct Bounds {
  U lower, upper;
  UX lower_state_jacobian = UX::Zero(), upper_state_jacobian = UX::Zero();
};

struct Obstacle {
  std::vector<U> centers;
  std::vector<U> footprint;
  double dt = 0.1;
  // Time of the MPC's first state relative to prediction publication stamp.
  double age = 0.0;
  U center(double future) const {
    if (centers.empty() || dt <= 0) throw std::invalid_argument("Invalid prediction");
    double index = std::max(0.0, (age + future) / dt);
    if (index > static_cast<double>(centers.size() - 1) + 1e-8)
      throw std::out_of_range("Prediction horizon exhausted");
    auto a = static_cast<size_t>(index);
    auto b = std::min(a + 1, centers.size() - 1);
    return centers[a] + (index - static_cast<double>(a)) * (centers[b] - centers[a]);
  }
  // Convex footprint signed distance; gradient points away from the obstacle.
  std::pair<double, U> distance(const U& point, double future) const {
    U q = point - center(future), closest = U::Zero();
    double best = std::numeric_limits<double>::infinity();
    bool positive = false, negative = false;
    if (footprint.size() < 3) throw std::invalid_argument("Footprint requires a polygon");
    for (size_t i = 0; i < footprint.size(); ++i) {
      U a = footprint[i], edge = footprint[(i + 1) % footprint.size()] - a;
      double len2 = edge.squaredNorm();
      if (len2 < 1e-16) continue;
      double cross = edge.x() * (q.y()-a.y()) - edge.y() * (q.x()-a.x());
      positive |= cross > 1e-10; negative |= cross < -1e-10;
      U candidate = a + std::clamp((q-a).dot(edge)/len2, 0.0, 1.0)*edge;
      double d = (q-candidate).squaredNorm();
      if (d < best) { best = d; closest = candidate; }
    }
    double d = std::sqrt(best);
    bool inside = !(positive && negative);
    U gradient = d > 1e-10 ? U((q-closest)/d) : U::UnitX();
    if (inside) return {-d, -gradient};
    return {d, gradient};
  }
};

struct Grid {
  int width = 0, height = 0;
  double resolution = 0.025;
  U origin = U::Zero();
  std::vector<unsigned char> cells;
  unsigned char at(const U& p) const {
    U c = (p-origin)/resolution;
    int x = static_cast<int>(std::floor(c.x())), y = static_cast<int>(std::floor(c.y()));
    if (x < 0 || y < 0 || x >= width || y >= height) return 255;
    return cells[static_cast<size_t>(y)*width+x];
  }
  // Lethal obstacles are already inflated by Nav2. Unknown cells are not:
  // check every cell intersecting the circular body, including map edges.
  bool body_known(const U& p,double radius) const {
    if(!p.allFinite()||!std::isfinite(radius)||radius<0||!std::isfinite(resolution)||resolution<=0||!origin.allFinite()||
       width<=0||height<=0||cells.size()!=size_t(width)*height)return false;
    U lo=(p-U::Constant(radius)-origin)/resolution;
    U hi=(p+U::Constant(radius)-origin)/resolution;
    if(lo.minCoeff()<0||hi.x()>=width||hi.y()>=height)return false;
    for(int y=int(std::floor(lo.y()));y<=int(std::floor(hi.y()));++y)
      for(int x=int(std::floor(lo.x()));x<=int(std::floor(hi.x()));++x){
        if(cells[size_t(y)*width+x]!=255)continue;
        U lower=origin+resolution*U(x,y),upper=lower+U::Constant(resolution);
        U nearest=p.cwiseMax(lower).cwiseMin(upper);
        if((p-nearest).squaredNorm()<=radius*radius+1e-12)return false;
      }
    return true;
  }
  std::pair<double, U> sample(const U& p) const {
    U c = (p-origin)/resolution - U::Constant(0.5);
    int ix = static_cast<int>(std::floor(c.x())), iy = static_cast<int>(std::floor(c.y()));
    if (ix < 0 || iy < 0 || ix+1 >= width || iy+1 >= height) return {1.0, U::Zero()};
    double dx = c.x()-ix, dy = c.y()-iy;
    auto get = [&](int x, int y) {return std::min(1.0, cells[static_cast<size_t>(y)*width+x]/253.0);};
    double a=get(ix,iy), b=get(ix+1,iy), d=get(ix,iy+1), e=get(ix+1,iy+1);
    return {(1-dy)*((1-dx)*a+dx*b)+dy*((1-dx)*d+dx*e),
      U(((1-dy)*(b-a)+dy*(e-d))/resolution,
        ((1-dx)*(d-a)+dx*(e-b))/resolution)};
  }
};

// State: map x/y, FAST-LIO estimated map vx/vy, previous target vx/vy,
// previous target increment. Control: next target map vx/vy.
// Lidar heading is an exogenous input. Chassis spin is NEVER used as lidar yaw rate.
class Problem {
public:
  Settings settings;
  std::array<U, N+1> positions{}, velocities{};
  std::array<double, N+1> lidar_yaws{};
  const Grid* grid = nullptr;
  std::vector<Obstacle> obstacles;

  void matrices(int k, XX& fx, XU& fu) const {
    double c=std::cos(lidar_yaws[k]), s=std::sin(lidar_yaws[k]);
    Eigen::Matrix2d r; r << c,-s,s,c;
    Eigen::Vector2d decay(std::exp(-DT/settings.tau_x), std::exp(-DT/settings.tau_y));
    Eigen::Matrix2d a=r*decay.asDiagonal()*r.transpose();
    Eigen::Vector2d integ(settings.tau_x*(1-decay.x()), settings.tau_y*(1-decay.y()));
    Eigen::Matrix2d b=r*integ.asDiagonal()*r.transpose();
    fx.setZero(); fu.setZero();
    fx.block<2,2>(0,0).setIdentity(); fx.block<2,2>(0,2)=b;
    fu.block<2,2>(0,0)=DT*Eigen::Matrix2d::Identity()-b;
    fx.block<2,2>(2,2)=a; fu.block<2,2>(2,0)=Eigen::Matrix2d::Identity()-a;
    fu.block<2,2>(4,0).setIdentity();
    fx.block<2,2>(6,4)=-Eigen::Matrix2d::Identity(); fu.block<2,2>(6,0).setIdentity();
  }
  X dynamics(int k, const X& x, const U& u) const {
    XX fx; XU fu; matrices(k,fx,fu); return fx*x+fu*u;
  }
  void dynamics_jacobians(int k,const X&,const U&,XX& fx,XU& fu) const {matrices(k,fx,fu);}

  Bounds control_bounds(int, const X& x) const {
    Bounds b;
    // Each axis may use the full speed; the exact
    // vector bound is penalized in optimization and checked on the full rollout.
    double a=settings.acceleration*DT/std::sqrt(2.0);
    double j=settings.jerk*DT*DT/std::sqrt(2.0);
    for (int i=0;i<2;++i) {
      std::array<double,3> lo{-settings.speed,x[4+i]-a,x[4+i]+x[6+i]-j};
      std::array<double,3> hi{ settings.speed,x[4+i]+a,x[4+i]+x[6+i]+j};
      int l=static_cast<int>(std::max_element(lo.begin(),lo.end())-lo.begin());
      int h=static_cast<int>(std::min_element(hi.begin(),hi.end())-hi.begin());
      b.lower[i]=lo[l]; b.upper[i]=hi[h];
      if(l>0)b.lower_state_jacobian(i,4+i)=1;
      if(h>0)b.upper_state_jacobian(i,4+i)=1;
      if(l==2)b.lower_state_jacobian(i,6+i)=1;
      if(h==2)b.upper_state_jacobian(i,6+i)=1;
    }
    return b;
  }

  void environment(int k,const X& x,double& cost,X& gradient,XX& hessian) const {
    auto residual=[&](double value,const U& jac,double w) {
      cost+=0.5*w*value*value;
      gradient.head<2>()+=w*value*jac;
      hessian.block<2,2>(0,0)+=w*jac*jac.transpose();
    };
    if(grid){auto [v,g]=grid->sample(x.head<2>());residual(v,g,settings.static_cost_weight);}
    for(const auto& obstacle:obstacles){
      auto [distance,g]=obstacle.distance(x.head<2>(),k*DT);
      double intrusion=settings.radius+settings.obstacle_margin-distance;
      if(intrusion>0)residual(intrusion,-g,settings.obstacle_weight);
    }
  }
  void costs(int k,const X& x,const U& u,double& cost,X& lx,U& lu,XX& lxx,UX& lux,Eigen::Matrix2d& luu) const {
    cost=0;lx.setZero();lu.setZero();lxx.setZero();lux.setZero();luu.setZero();
    auto state=[&](int offset,const U& target,double weight) {
      U error=x.segment<2>(offset)-target;
      cost+=0.5*weight*error.squaredNorm();lx.segment<2>(offset)+=weight*error;
      lxx.block<2,2>(offset,offset)+=weight*Eigen::Matrix2d::Identity();
    };
    state(0,positions[k],settings.position_weight*(k==N?3:1));
    state(2,velocities[k],settings.velocity_weight);
    environment(k,x,cost,lx,lxx);
    if(k==N)return;
    cost+=0.5*settings.command_weight*u.squaredNorm();
    lu+=settings.command_weight*u;luu+=settings.command_weight*Eigen::Matrix2d::Identity();
    // Target increments and their changes correspond to target acceleration/jerk.
    for(int order=1;order<=2;++order){
      U error=u-x.segment<2>(4);UX jac=UX::Zero();jac.block<2,2>(0,4)=-Eigen::Matrix2d::Identity();
      double w=settings.acceleration_weight/(DT*DT);
      if(order==2){error-=x.segment<2>(6);jac.block<2,2>(0,6)=-Eigen::Matrix2d::Identity();w=settings.jerk_weight/(DT*DT);}
      cost+=0.5*w*error.squaredNorm();lx+=w*jac.transpose()*error;lu+=w*error;
      lxx+=w*jac.transpose()*jac;lux+=w*jac;luu+=w*Eigen::Matrix2d::Identity();
    }
    double speed=u.norm(), excess=speed-settings.speed*0.98;
    if(excess>0){U g=u/speed;double w=10000;
      cost+=0.5*w*excess*excess;lu+=w*excess*g;luu+=w*g*g.transpose();}
  }
  double running_cost(int k,const X& x,const U& u) const {double c;X gx;U gu;XX xx;UX ux;Eigen::Matrix2d uu;costs(k,x,u,c,gx,gu,xx,ux,uu);return c;}
  void running_cost_derivatives(int k,const X& x,const U& u,X& lx,U& lu,XX& lxx,UX& lux,Eigen::Matrix2d& luu) const {double c;costs(k,x,u,c,lx,lu,lxx,lux,luu);}
  double terminal_cost(const X& x) const{return running_cost(N,x,U::Zero());}
  void terminal_cost_derivatives(const X& x,X& lx,XX& lxx) const {double c;U lu;UX ux;Eigen::Matrix2d uu;costs(N,x,U::Zero(),c,lx,lu,lxx,ux,uu);}

  bool safe(const std::array<X,N+1>& xs,const std::array<U,N>& us) const {
    for(int k=0;k<N;++k){
      if(!xs[k].allFinite()||!us[k].allFinite()||us[k].norm()>settings.speed+1e-7)return false;
      double step=grid?grid->resolution*0.4:0.01;
      int samples=std::max(1,static_cast<int>(std::ceil((xs[k+1].head<2>()-xs[k].head<2>()).norm()/step)));
      // Also sample time so a moving obstacle cannot tunnel through a stationary robot.
      samples=std::max(samples,5);
      for(int n=0;n<=samples;++n){
        double f=static_cast<double>(n)/samples;
        U point=(1-f)*xs[k].head<2>()+f*xs[k+1].head<2>();
        double sweep_guard=(xs[k+1].head<2>()-xs[k].head<2>()).norm()/(2*samples);
        if(grid && (grid->at(point)>=253||!grid->body_known(point,settings.radius+sweep_guard)))return false;
        for(const auto& obstacle:obstacles)
          if(obstacle.distance(point,(k+f)*DT).first<=settings.radius)return false;
      }
    }
    return xs[N].allFinite();
  }
};
} // namespace hw_omni
