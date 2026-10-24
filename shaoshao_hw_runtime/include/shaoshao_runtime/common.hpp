#pragma once
#include <Eigen/Dense>
#include <json/json.h>
#include <yaml-cpp/yaml.h>
#include <openssl/evp.h>
#include <openssl/rand.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <unistd.h>

namespace hw_runtime {
namespace fs=std::filesystem;
using Pose=Eigen::Isometry3d;using Point=Eigen::Vector3d;using Cloud=std::vector<Point>;
constexpr double pi=3.14159265358979323846;
inline double steady(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
inline std::string read_text(const fs::path& path,size_t limit=32*1024*1024){
  std::ifstream f(path,std::ios::binary);if(!f)throw std::runtime_error("Cannot read "+path.string());
  auto length=fs::file_size(path);if(length>limit)throw std::runtime_error("File exceeds size budget");
  return std::string(std::istreambuf_iterator<char>(f),{});
}
inline void atomic_write(const fs::path& path,const std::string& data){
  fs::create_directories(path.parent_path());auto temporary=path.string()+".tmp-"+std::to_string(getpid());
  {std::ofstream out(temporary,std::ios::binary|std::ios::trunc);out.write(data.data(),static_cast<std::streamsize>(data.size()));if(!out)throw std::runtime_error("Cannot write "+temporary);}
  fs::rename(temporary,path);
}
inline std::string json_text(const Json::Value& value){Json::StreamWriterBuilder b;b["indentation"]="";return Json::writeString(b,value);}
inline Json::Value json_parse(const std::string& text){
  Json::CharReaderBuilder b;b["rejectDupKeys"]=true;b["failIfExtra"]=true;auto r=std::unique_ptr<Json::CharReader>(b.newCharReader());
  Json::Value out;std::string errors;if(!r->parse(text.data(),text.data()+text.size(),&out,&errors))throw std::invalid_argument("Invalid JSON");return out;
}
inline Json::Value read_json(const fs::path& p){return json_parse(read_text(p));}
inline void write_json(const fs::path& p,const Json::Value& v){atomic_write(p,json_text(v)+"\n");}
inline void write_yaml(const fs::path& p,const YAML::Node& v){YAML::Emitter e;e.SetDoublePrecision(17);e<<v;if(!e.good())throw std::runtime_error("YAML serialization failed");atomic_write(p,std::string(e.c_str())+"\n");}
inline fs::path runtime_path(){const char* value=std::getenv("SHAOSHAO_HW_RUNTIME");if(value&&*value)return fs::absolute(value);const char* home=std::getenv("HOME");if(!home)throw std::runtime_error("HOME is unavailable");return fs::path(home)/"ccc/shaoshao-hw-runtime";}
inline fs::path robot_configuration(const fs::path& runtime){auto snapshot=runtime/"active_robot.yaml";return fs::is_regular_file(snapshot)?snapshot:runtime/"shaoshao/web_panel/robot.yaml";}
inline std::string random_token(){std::array<unsigned char,24> bytes;if(RAND_bytes(bytes.data(),bytes.size())!=1)throw std::runtime_error("Random generator failed");std::ostringstream out;for(auto b:bytes)out<<std::hex<<std::setfill('0')<<std::setw(2)<<int(b);return out.str();}
inline std::string sha256(const fs::path& path){
  std::ifstream f(path,std::ios::binary);if(!f)throw std::runtime_error("Cannot hash map");
  auto context=std::unique_ptr<EVP_MD_CTX,decltype(&EVP_MD_CTX_free)>(EVP_MD_CTX_new(),EVP_MD_CTX_free);
  if(!context||EVP_DigestInit_ex(context.get(),EVP_sha256(),nullptr)!=1)throw std::runtime_error("SHA256 init failed");
  std::array<char,65536> data;while(f){f.read(data.data(),data.size());if(f.gcount()&&EVP_DigestUpdate(context.get(),data.data(),f.gcount())!=1)throw std::runtime_error("SHA256 update failed");}
  if(!f.eof())throw std::runtime_error("Map read failed");std::array<unsigned char,EVP_MAX_MD_SIZE> digest;unsigned length=0;
  if(EVP_DigestFinal_ex(context.get(),digest.data(),&length)!=1)throw std::runtime_error("SHA256 failed");
  std::ostringstream out;for(unsigned i=0;i<length;++i)out<<std::hex<<std::setfill('0')<<std::setw(2)<<int(digest[i]);return out.str();
}
inline Pose pose(double x=0,double y=0,double yaw=0){Pose p=Pose::Identity();p.translation()<<x,y,0;p.linear()=Eigen::AngleAxisd(yaw,Point::UnitZ()).toRotationMatrix();return p;}
inline std::pair<double,double> delta(const Pose& a,const Pose& b){return {(a.translation()-b.translation()).norm(),Eigen::AngleAxisd(a.linear().transpose()*b.linear()).angle()};}
inline Pose interpolate(const Pose& a,const Pose& b,double alpha){Pose p=Pose::Identity();p.translation()=a.translation()+alpha*(b.translation()-a.translation());p.linear()=Eigen::Quaterniond(a.linear()).slerp(alpha,Eigen::Quaterniond(b.linear())).toRotationMatrix();return p;}
inline Json::Value point_json(const Point& p){Json::Value v(Json::arrayValue);for(int i=0;i<3;++i)v.append(p[i]);return v;}
inline Point yaml_point(const YAML::Node& n){if(!n.IsSequence()||n.size()!=3)throw std::invalid_argument("XYZ vector required");Point p(n[0].as<double>(),n[1].as<double>(),n[2].as<double>());if(!p.allFinite())throw std::invalid_argument("Nonfinite XYZ");return p;}
inline double finite_number(const Json::Value& object,const std::string& key){const auto& value=object[key];if(!value.isNumeric()||!std::isfinite(value.asDouble()))throw std::invalid_argument("Finite numeric "+key+" required");return value.asDouble();}
inline uint64_t sequence_number(const Json::Value& object,const std::string& key="seq"){const auto& v=object[key];if(!v.isUInt64()||v.asUInt64()>9007199254740992ULL)throw std::invalid_argument("Invalid sequence");return v.asUInt64();}
struct RegistrationSettings {
  double period=1,voxel=.05,max_error=.30,min_overlap=.80,min_eigenvalue=1000,max_condition=1e6,max_step=.2,max_angle=.1,alpha=.4;
  int confirmations=3;double fresh=1.6,timeout=2,hold_distance=.05,degraded_speed=.05,search_timeout=30,retry=5,scan_timeout=.5;
};
struct IndexSettings {double distance=.25,angle=pi/18,radius=15;int rings=20,sectors=60;};
struct Config {
  YAML::Node yaml;RegistrationSettings registration;IndexSettings index;
  double speed=.5,resolution=.025,radius=.297658,lidar_height=.327,acceleration=.3,deceleration=.3,jerk=1;
  double tau_x=.3,tau_y=.3,feedback_timeout=.15,prediction_timeout=.3,obstacle_margin=.15,lio_timeout=.15,lio_velocity_std=.5;
  Point lidar_offset=Point(-.011,-.02329,.04412),center_offset=Point(.00318,-.02329,0);
  explicit Config(const fs::path& path):yaml(YAML::LoadFile(path.string())){
    if(!yaml.IsMap())throw std::invalid_argument("robot.yaml must be a mapping");auto nav=yaml["navigation"];
    auto number=[](YAML::Node section,const char* name,double fallback){double v=section[name]?section[name].as<double>():fallback;if(!std::isfinite(v)||v<=0)throw std::invalid_argument(std::string("Invalid ")+name);return v;};
    speed=number(nav,"max_speed_mps",.5);resolution=number(nav,"map_resolution_m",.025);
    radius=number(yaml["robot"],"circumscribed_radius_m",.297658);lidar_height=number(yaml["robot"],"lidar_height_m",.327);
    auto motion=nav["motion"];acceleration=number(motion,"acceleration_mps2",.3);deceleration=number(motion,"deceleration_mps2",.3);jerk=number(motion,"jerk_mps3",1.);
    if(motion["frequency_hz"]&&motion["frequency_hz"].as<double>()!=50)throw std::invalid_argument("Output frequency must remain 50 Hz");
    if(motion["command_timeout_s"]&&number(motion,"command_timeout_s",.2)>.2)throw std::invalid_argument("Invalid command watchdog");
    auto m=nav["mpc"];tau_x=number(m,"tau_x_s",.3);tau_y=number(m,"tau_y_s",.3);feedback_timeout=number(m,"authority_timeout_s",number(m,"feedback_timeout_s",.15));lio_timeout=number(m,"lio_timeout_s",.15);lio_velocity_std=number(m,"lio_velocity_std_limit_mps",.5);prediction_timeout=number(m,"prediction_timeout_s",.3);obstacle_margin=number(m,"obstacle_margin_m",.15);
    auto r=nav["registration"];auto& p=registration;
    p.period=number(r,"period_s",1);p.voxel=number(r,"voxel_m",.05);p.max_error=number(r,"max_error",.3);p.min_overlap=number(r,"min_overlap",.8);p.min_eigenvalue=number(r,"min_eigenvalue",1000);p.max_condition=number(r,"max_condition",1e6);
    p.max_step=number(r,"max_step_m",.2);p.max_angle=number(r,"max_step_rad",.1);p.alpha=number(r,"alpha",.4);
    if(r["confirmations"])p.confirmations=r["confirmations"].as<int>();p.fresh=number(r,"fresh_s",1.6);p.timeout=number(r,"timeout_s",2);p.hold_distance=number(r,"hold_distance_m",.05);p.degraded_speed=number(r,"degraded_speed_mps",.05);p.search_timeout=number(r,"search_timeout_s",30);p.retry=number(r,"retry_s",5);p.scan_timeout=number(r,"scan_timeout_s",.5);
    auto i=nav["index"];index.distance=number(i,"distance_m",.25);index.angle=number(i,"angle_rad",pi/18);index.radius=number(i,"radius_m",15);if(i["rings"])index.rings=i["rings"].as<int>();if(i["sectors"])index.sectors=i["sectors"].as<int>();
    if(speed<.01||speed>.5||resolution<.01||resolution>.1||tau_x<.02||tau_x>3||tau_y<.02||tau_y>3||feedback_timeout>.2||lio_timeout>.15||lio_velocity_std>2||prediction_timeout>.5||p.alpha>1||p.min_overlap>1||p.confirmations<2||p.confirmations>10||p.fresh>=p.timeout||p.timeout>5||p.hold_distance>.2||p.degraded_speed>.1||index.rings<1||index.rings>100||index.sectors<6||index.sectors>360)throw std::invalid_argument("Configuration safety/resource bounds exceeded");
    auto reference=nav["reference_point"]?nav["reference_point"].as<std::string>():"yaw_axis";if(reference!="yaw_axis")throw std::invalid_argument("Native runtime requires the measured yaw_axis reference");
    auto offset=yaml["yaw"]["lidar_offset_xy_m"];if(!offset.IsSequence()||offset.size()!=2)throw std::invalid_argument("Measured lidar XY offset required");
    Eigen::Vector2d xy(offset[0].as<double>(),offset[1].as<double>());double length=yaml["yaw"]["lidar_offset_radius_m"].as<double>();
    if(!xy.allFinite()||!std::isfinite(length)||std::abs(xy.norm()-length)>1e-6)throw std::invalid_argument("Inconsistent lidar eccentricity");
    center_offset=Point(lidar_offset.x()-xy.x(),lidar_offset.y()-xy.y(),0);
  }
};
}
