#pragma once
#include "npz.hpp"
#include <numeric>

namespace hw_runtime {
class KDTree {
public:
  explicit KDTree(Cloud points):points_(std::move(points)){if(points_.empty())throw std::invalid_argument("Empty static map");std::vector<size_t> ids(points_.size());std::iota(ids.begin(),ids.end(),0);root_=build(ids,0,ids.size(),0);}
  double distance(const Point& point) const {double best=INFINITY;query(root_,point,best);return std::sqrt(best);}
private:
  struct Node {size_t point;int left=-1,right=-1,axis=0;};Cloud points_;std::vector<Node> nodes_;int root_;
  int build(std::vector<size_t>& ids,size_t begin,size_t end,int axis){if(begin==end)return -1;size_t middle=(begin+end)/2;
    std::nth_element(ids.begin()+begin,ids.begin()+middle,ids.begin()+end,[&](size_t a,size_t b){return points_[a][axis]<points_[b][axis];});
    int id=static_cast<int>(nodes_.size());nodes_.push_back({ids[middle],-1,-1,axis});int left=build(ids,begin,middle,(axis+1)%3),right=build(ids,middle+1,end,(axis+1)%3);nodes_[id].left=left;nodes_[id].right=right;return id;
  }
  void query(int index,const Point& p,double& best) const {if(index<0)return;const auto& node=nodes_[index];best=std::min(best,(points_[node.point]-p).squaredNorm());double split=p[node.axis]-points_[node.point][node.axis];query(split<0?node.left:node.right,p,best);if(split*split<best)query(split<0?node.right:node.left,p,best);}
};
inline std::vector<double> descriptor(const Cloud& input,const IndexSettings& settings){
  Cloud points;std::vector<double> heights;
  for(const auto& p:input){double radius=p.head<2>().norm();if(p.allFinite()&&radius>=.2&&radius<settings.radius){points.push_back(p);heights.push_back(p.z());}}
  std::vector<double> result(static_cast<size_t>(settings.rings)*settings.sectors,0);if(points.empty())return result;
  std::sort(heights.begin(),heights.end());double q=.05*(heights.size()-1);size_t low=static_cast<size_t>(q);double floor=heights[low]+(q-low)*(heights[std::min(low+1,heights.size()-1)]-heights[low]);
  for(const auto& p:points){int ring=std::min(settings.rings-1,static_cast<int>(p.head<2>().norm()/settings.radius*settings.rings));int sector=static_cast<int>((std::atan2(p.y(),p.x())+pi)/(2*pi)*settings.sectors)%settings.sectors;auto& value=result[ring*settings.sectors+sector];value=std::max(value,std::clamp(p.z()-floor+.1,.01,4.));}return result;
}
inline std::pair<double,int> descriptor_distance(const std::vector<double>& query,const std::vector<double>& target,const IndexSettings& p){
  std::pair<double,int> best{INFINITY,0};
  for(int shift=0;shift<p.sectors;++shift){double similarity=0;int count=0;
    for(int sector=0;sector<p.sectors;++sector){double dot=0,qnorm=0,tnorm=0;int moved=(sector-shift+p.sectors)%p.sectors;
      for(int ring=0;ring<p.rings;++ring){double q=query[ring*p.sectors+sector],t=target[ring*p.sectors+moved];dot+=q*t;qnorm+=q*q;tnorm+=t*t;}
      if(qnorm>0&&tnorm>0){similarity+=dot/std::sqrt(qnorm*tnorm);++count;}
    }
    if(count>=6){double distance=1-similarity/count;if(distance<best.first)best={distance,shift};}
  }return best;
}
inline Json::Value index_settings_json(const IndexSettings& p){Json::Value j;j["distance_m"]=p.distance;j["angle_rad"]=p.angle;j["rings"]=p.rings;j["sectors"]=p.sectors;j["radius_m"]=p.radius;return j;}
class IndexWriter {
public:
  IndexWriter(fs::path directory,IndexSettings settings={}):root_(std::move(directory)),settings_(settings){fs::create_directories(root_);}
  size_t size() const{return frames_.size();}
  bool add(const Cloud& points,const Pose& sensor,int64_t stamp){
    if(points.size()<500)return false;if(last_){auto [d,a]=delta(*last_,sensor);if(d<settings_.distance&&a<settings_.angle)return false;}
    std::ostringstream name;name<<"frame-"<<std::setfill('0')<<std::setw(6)<<frames_.size()<<".npz";
    write_npz(root_/name.str(),{{"points",cloud_array(points)},{"pose",pose_array(sensor)}},true);
    Json::Value frame;frame["file"]=name.str();frame["stamp_ns"]=Json::Int64(stamp);frames_.push_back(frame);last_=sensor;manifest(false,"");return true;
  }
  void finalize(const fs::path& map_path,const Cloud& static_points){
    if(static_points.size()<500)throw std::runtime_error("Static map too sparse for global localization");KDTree tree(static_points);
    Array contexts,poses;std::vector<Json::Value> kept;
    for(const auto& frame:frames_){auto path=root_/frame["file"].asString();auto points=array_cloud(read_npz(path,"points"));auto sensor=array_pose(read_npz(path,"pose"));Cloud filtered;
      for(const auto& p:points)if(tree.distance(sensor*p)<=.1)filtered.push_back(p);
      if(filtered.size()<500)continue;auto context=descriptor(filtered,settings_);contexts.data.insert(contexts.data.end(),context.begin(),context.end());auto a=pose_array(sensor);poses.data.insert(poses.data.end(),a.data.begin(),a.data.end());kept.push_back(frame);
    }
    if(kept.empty())throw std::runtime_error("No static keyframes remain after map filtering");
    contexts.shape={kept.size(),size_t(settings_.rings),size_t(settings_.sectors)};poses.shape={kept.size(),4,4};
    write_npz(root_/"index.npz",{{"contexts",contexts},{"poses",poses}});frames_=std::move(kept);manifest(true,sha256(map_path));
  }
private:
  fs::path root_;IndexSettings settings_;std::vector<Json::Value> frames_;std::optional<Pose> last_;
  void manifest(bool complete,const std::string& hash){Json::Value m;m["version"]=1;m["complete"]=complete;m["settings"]=index_settings_json(settings_);m["map_sha256"]=hash.empty()?Json::Value():Json::Value(hash);m["frames"]=Json::Value(Json::arrayValue);for(const auto& f:frames_)m["frames"].append(f);write_json(root_/"manifest.json",m);}
};
class RetrievalIndex {
public:
  RetrievalIndex(const fs::path& directory,const fs::path& map){
    auto manifest=read_json(directory/"manifest.json");if(manifest["version"].asInt()!=1||!manifest["complete"].asBool())throw std::runtime_error("Map index is not finalized");
    if(manifest["map_sha256"].asString()!=sha256(map))throw std::runtime_error("Map/index hash mismatch");auto s=manifest["settings"];
    settings_.rings=s["rings"].asInt();settings_.sectors=s["sectors"].asInt();settings_.radius=finite_number(s,"radius_m");
    if(settings_.rings<1||settings_.rings>100||settings_.sectors<6||settings_.sectors>360||settings_.radius<=0)throw std::runtime_error("Invalid index settings");
    auto contexts=read_npz(directory/"index.npz","contexts"),poses=read_npz(directory/"index.npz","poses");
    if(contexts.shape.size()!=3||poses.shape.size()!=3||contexts.shape[0]!=poses.shape[0]||contexts.shape[1]!=size_t(settings_.rings)||contexts.shape[2]!=size_t(settings_.sectors)||poses.shape[1]!=4||poses.shape[2]!=4)throw std::runtime_error("Invalid map index dimensions");
    for(size_t i=0;i<contexts.shape[0];++i){size_t count=size_t(settings_.rings)*settings_.sectors;contexts_.emplace_back(contexts.data.begin()+i*count,contexts.data.begin()+(i+1)*count);poses_.push_back(array_pose(poses,i*16));std::vector<double> key(settings_.rings);for(int ring=0;ring<settings_.rings;++ring)for(int sector=0;sector<settings_.sectors;++sector)key[ring]+=contexts_.back()[ring*settings_.sectors+sector]/settings_.sectors;keys_.push_back(key);}
  }
  std::vector<Pose> candidates(const Cloud& points,const Pose& current_sensor) const {
    auto query=descriptor(points,settings_);std::vector<double> key(settings_.rings);
    for(int ring=0;ring<settings_.rings;++ring)for(int sector=0;sector<settings_.sectors;++sector)key[ring]+=query[ring*settings_.sectors+sector]/settings_.sectors;
    std::vector<std::pair<double,size_t>> order;
    for(size_t i=0;i<keys_.size();++i){double distance=0;for(int ring=0;ring<settings_.rings;++ring)distance+=std::pow(key[ring]-keys_[i][ring],2);order.emplace_back(distance,i);}std::stable_sort(order.begin(),order.end());
    std::vector<std::pair<double,Pose>> ranked;
    for(size_t i=0;i<std::min(size_t(10),order.size());++i){size_t index=order[i].second;auto [distance,shift]=descriptor_distance(query,contexts_[index],settings_);if(std::isfinite(distance))ranked.emplace_back(distance,poses_[index]*pose(0,0,-shift*2*pi/settings_.sectors)*current_sensor.inverse());}
    std::stable_sort(ranked.begin(),ranked.end(),[](const auto& a,const auto& b){return a.first<b.first;});std::vector<Pose> out;
    for(const auto& [distance,candidate]:ranked){(void)distance;bool duplicate=false;for(const auto& existing:out){auto [d,a]=delta(candidate,existing);if(d<.5&&a<.25){duplicate=true;break;}}if(!duplicate)out.push_back(candidate);if(out.size()==3)break;}return out;
  }
private:
  IndexSettings settings_;std::vector<std::vector<double>> contexts_,keys_;std::vector<Pose> poses_;
};
}
