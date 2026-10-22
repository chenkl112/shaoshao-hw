#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <optional>
#include <queue>
#include <set>
#include <stdexcept>
#include <vector>

namespace hw_obstacles {
using Point=Eigen::Vector2d;
using CloudPoint=Eigen::Vector3d;
using Polygon=std::vector<Point>;
constexpr double PI=3.14159265358979323846;
inline double cross(const Point& a,const Point& b){return a.x()*b.y()-a.y()*b.x();}

inline Polygon hull(Polygon points,double padding=.04){
  if(points.empty()||!std::isfinite(padding)||padding<=0)throw std::invalid_argument("Invalid obstacle shape");
  Polygon expanded;expanded.reserve(points.size()*8);
  for(const auto& point:points){
    if(!point.allFinite())throw std::invalid_argument("Nonfinite footprint point");
    for(int i=0;i<8;++i){double a=i*PI/4;expanded.push_back(point+padding/std::cos(PI/8)*Point(std::cos(a),std::sin(a)));}
  }
  std::sort(expanded.begin(),expanded.end(),[](const Point& a,const Point& b){return a.x()<b.x()||(a.x()==b.x()&&a.y()<b.y());});
  expanded.erase(std::unique(expanded.begin(),expanded.end(),[](const Point& a,const Point& b){return (a-b).squaredNorm()<1e-20;}),expanded.end());
  Polygon result;
  for(const auto& p:expanded){while(result.size()>1&&cross(result.back()-result[result.size()-2],p-result.back())<=1e-12)result.pop_back();result.push_back(p);}
  size_t lower=result.size();
  for(auto it=expanded.rbegin()+1;it!=expanded.rend();++it){while(result.size()>lower&&cross(result.back()-result[result.size()-2],*it-result.back())<=1e-12)result.pop_back();result.push_back(*it);}
  result.pop_back();return result;
}

struct Detection {Point center=Point::Zero();Polygon footprint;};
struct Prediction {uint64_t id=0;bool confirmed=false;Polygon footprint;std::vector<Point> centers;};
struct Settings {
  double resolution=.05, range=8., static_distance=.15, association_gate=.8;
  double lost_timeout=.5, process_noise=1.,measurement_noise=.05,velocity_decay=2.;
  int confirmations=3;
  void validate() const {
    for(double value:{resolution,range,static_distance,association_gate,lost_timeout,process_noise,measurement_noise,velocity_decay})
      if(!std::isfinite(value)||value<=0)throw std::invalid_argument("Invalid obstacle setting");
    if(resolution<.02||resolution>.2||range<1||range>15||confirmations<2||confirmations>10||lost_timeout>2)
      throw std::invalid_argument("Obstacle setting outside resource bounds");
  }
};

// The nearest-static-map query is supplied by the ROS adapter (PCL KD-tree).
// Keeping the core independent of ROS/PCL permits native regression tests.
class Detector {
public:
  explicit Detector(Settings settings={}):settings_(settings){settings_.validate();}
  std::vector<Detection> detect(const std::vector<CloudPoint>& points,const Point& center,
      double ground_z,const std::function<double(const CloudPoint&)>& nearest) const {
    if(points.size()>200000||!center.allFinite()||!std::isfinite(ground_z))throw std::invalid_argument("Invalid obstacle cloud");
    int side=static_cast<int>(std::ceil(2*settings_.range/settings_.resolution))+1;
    Point origin=center-Point::Constant(settings_.range);
    std::vector<Point> dynamic;std::vector<int> indices;
    std::vector<uint8_t> occupied(static_cast<size_t>(side)*side,0);
    for(const auto& p:points){
      if(!p.allFinite()||p.z()<=ground_z+.1||p.z()>=ground_z+1.5||
         (p.head<2>()-center).norm()>=settings_.range)continue;
      double distance=nearest(p);
      if(std::isnan(distance)||distance<0)throw std::invalid_argument("Invalid nearest-map distance");
      if(distance<=settings_.static_distance)continue;
      int x=static_cast<int>(std::floor((p.x()-origin.x())/settings_.resolution));
      int y=static_cast<int>(std::floor((p.y()-origin.y())/settings_.resolution));
      int index=y*side+x;occupied[index]=1;dynamic.push_back(p.head<2>());indices.push_back(index);
    }
    // 3x3 binary closing followed by eight-connected components.
    auto morphology=[&](const std::vector<uint8_t>& input,bool dilate){
      std::vector<uint8_t> output(input.size(),0);
      for(int y=0;y<side;++y)for(int x=0;x<side;++x){bool value=!dilate;
        for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx){int nx=x+dx,ny=y+dy;
          bool bit=nx>=0&&ny>=0&&nx<side&&ny<side&&input[ny*side+nx];
          value=dilate?(value||bit):(value&&bit);
        }
        output[y*side+x]=value;
      }
      return output;
    };
    auto closed=morphology(morphology(occupied,true),false);
    std::vector<int> labels(occupied.size(),0);int label=0;
    for(int root=0;root<side*side;++root){
      if(!closed[root]||labels[root])continue;
      ++label;std::queue<int> pending;pending.push(root);labels[root]=label;
      while(!pending.empty()){int index=pending.front();pending.pop();int x=index%side,y=index/side;
        for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx){int nx=x+dx,ny=y+dy;
          if(nx<0||ny<0||nx>=side||ny>=side)continue;
          int next=ny*side+nx;if(closed[next]&&!labels[next]){labels[next]=label;pending.push(next);}
        }
      }
    }
    std::map<int,Polygon> components;
    for(size_t i=0;i<dynamic.size();++i)if(labels[indices[i]])components[labels[indices[i]]].push_back(dynamic[i]);
    std::vector<Detection> detections;
    for(const auto& [id,cluster]:components){
      (void)id;if(cluster.size()<3)continue;
      Detection d;for(const auto& p:cluster)d.center+=p;d.center/=cluster.size();
      std::set<std::pair<int,int>> voxels;
      for(const auto& p:cluster){Point cell=(p-d.center)/settings_.resolution;voxels.emplace(static_cast<int>(std::floor(cell.x())),static_cast<int>(std::floor(cell.y())));}
      Polygon shape;for(const auto& [x,y]:voxels)shape.emplace_back((x+.5)*settings_.resolution,(y+.5)*settings_.resolution);
      d.footprint=hull(std::move(shape),settings_.resolution);detections.push_back(std::move(d));
    }
    if(detections.size()>64)throw std::runtime_error("Obstacle detection budget exceeded");
    return detections;
  }
private:
  Settings settings_;
};

// Rectangular Hungarian assignment, with a private unmatched option per row.
inline std::vector<int> associate(const std::vector<std::vector<double>>& distances,double gate){
  int rows=static_cast<int>(distances.size());if(!rows)return {};
  int detections=static_cast<int>(distances[0].size()),cols=detections+rows;
  std::vector<double> u(rows+1),v(cols+1);std::vector<int> assigned(cols+1),way(cols+1);
  for(int i=1;i<=rows;++i){
    assigned[0]=i;int j0=0;std::vector<double> minimum(cols+1,std::numeric_limits<double>::infinity());std::vector<bool> used(cols+1);
    do{
      used[j0]=true;int i0=assigned[j0],j1=0;double delta=std::numeric_limits<double>::infinity();
      for(int j=1;j<=cols;++j)if(!used[j]){
        double cost=j<=detections?(distances[i0-1][j-1]<gate?distances[i0-1][j-1]:10*gate):gate;
        double current=cost-u[i0]-v[j];if(current<minimum[j]){minimum[j]=current;way[j]=j0;}
        if(minimum[j]<delta){delta=minimum[j];j1=j;}
      }
      for(int j=0;j<=cols;++j)if(used[j]){u[assigned[j]]+=delta;v[j]-=delta;}else minimum[j]-=delta;
      j0=j1;
    }while(assigned[j0]);
    do{int previous=way[j0];assigned[j0]=assigned[previous];j0=previous;}while(j0);
  }
  std::vector<int> result(rows,-1);
  for(int j=1;j<=detections;++j)if(assigned[j]&&distances[assigned[j]-1][j-1]<gate)result[assigned[j]-1]=j-1;
  return result;
}

struct Track {
  uint64_t id=0;Eigen::Vector4d state=Eigen::Vector4d::Zero();Eigen::Matrix4d covariance=Eigen::Matrix4d::Identity();
  Polygon footprint;int hits=1;bool confirmed=false;double lost_s=0;
};
class Tracker {
public:
  explicit Tracker(Settings settings={}):settings_(settings){settings_.validate();}
  void reset(){tracks_.clear();stamp_.reset();}
  const std::vector<Track>& tracks() const{return tracks_;}
  void update(const std::vector<Detection>& detections,double stamp){
    if(!std::isfinite(stamp)||(stamp_&&stamp<=*stamp_))throw std::invalid_argument("Obstacle timestamps must advance");
    if(detections.size()>64)throw std::invalid_argument("Too many detections");
    for(const auto& d:detections)if(!d.center.allFinite()||d.footprint.size()<3||d.footprint.size()>512)
      throw std::invalid_argument("Invalid detection shape");
    double dt=stamp_?stamp-*stamp_:0;
    if(dt>settings_.lost_timeout){reset();dt=0;}stamp_=stamp;
    Eigen::Matrix4d f=Eigen::Matrix4d::Identity();f.block<2,2>(0,2)=dt*Eigen::Matrix2d::Identity();
    Eigen::Matrix<double,4,2> g;g.topRows<2>()=.5*dt*dt*Eigen::Matrix2d::Identity();g.bottomRows<2>()=dt*Eigen::Matrix2d::Identity();
    for(auto& t:tracks_){t.state=f*t.state;t.covariance=f*t.covariance*f.transpose()+settings_.process_noise*settings_.process_noise*g*g.transpose();t.lost_s+=dt;}
    std::vector<std::vector<double>> distances(tracks_.size(),std::vector<double>(detections.size()));
    for(size_t i=0;i<tracks_.size();++i)for(size_t j=0;j<detections.size();++j)distances[i][j]=(tracks_[i].state.head<2>()-detections[j].center).norm();
    auto assignments=associate(distances,settings_.association_gate);std::vector<bool> used(detections.size());
    Eigen::Matrix<double,2,4> h=Eigen::Matrix<double,2,4>::Zero();h.leftCols<2>().setIdentity();
    Eigen::Matrix2d noise=std::pow(settings_.measurement_noise,2)*Eigen::Matrix2d::Identity();
    for(size_t i=0;i<tracks_.size();++i){auto& t=tracks_[i];int j=assignments[i];if(j<0){t.hits=0;continue;}used[j]=true;
      Eigen::Matrix2d innovation=h*t.covariance*h.transpose()+noise;
      Eigen::Matrix<double,4,2> gain=innovation.ldlt().solve(h*t.covariance).transpose();
      t.state+=gain*(detections[j].center-h*t.state);
      Eigen::Matrix4d residual=Eigen::Matrix4d::Identity()-gain*h;
      t.covariance=residual*t.covariance*residual.transpose()+gain*noise*gain.transpose();
      t.footprint=detections[j].footprint;for(auto& p:t.footprint)p+=detections[j].center-t.state.head<2>();
      ++t.hits;t.confirmed=t.confirmed||t.hits>=settings_.confirmations;t.lost_s=0;
    }
    std::erase_if(tracks_,[&](const Track& t){return t.lost_s>settings_.lost_timeout;});
    for(size_t j=0;j<detections.size();++j)if(!used[j]){
      Track t;t.id=++next_id_;t.state.head<2>()=detections[j].center;t.footprint=detections[j].footprint;
      t.covariance.diagonal()<<std::pow(settings_.measurement_noise,2),std::pow(settings_.measurement_noise,2),1.,1.;tracks_.push_back(std::move(t));
    }
    if(tracks_.size()>128)throw std::runtime_error("Retained obstacle budget exceeded");
  }
  std::vector<Prediction> predict(double horizon=3.,double dt=.1) const {
    if(!std::isfinite(horizon)||!std::isfinite(dt)||horizon<=0||horizon>5||dt<.02||dt>.2)
      throw std::invalid_argument("Invalid prediction horizon");
    std::vector<Prediction> result;
    for(const auto& t:tracks_){
      Prediction p;p.id=t.id;p.confirmed=t.confirmed;
      p.footprint=hull(t.footprint,.03+t.lost_s*std::max(.2,t.state.tail<2>().norm()));
      Point velocity=t.confirmed?Point(t.state.tail<2>()):Point::Zero();
      for(int i=0;i<=static_cast<int>(std::ceil(horizon/dt));++i){double d=settings_.velocity_decay*(1-std::exp(-i*dt/settings_.velocity_decay));p.centers.push_back(t.state.head<2>()+d*velocity);}
      result.push_back(std::move(p));
    }
    return result;
  }
private:
  Settings settings_;std::vector<Track> tracks_;std::optional<double> stamp_;uint64_t next_id_=0;
};
}
