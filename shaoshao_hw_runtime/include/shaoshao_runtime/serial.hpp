#pragma once
#include "common.hpp"
#include <bit>
#include <map>
#include <set>

namespace hw_runtime::serial {
using Values=std::array<float,4>;
inline uint16_t crc(const unsigned char* data,size_t length){uint16_t value=0xffff;for(size_t i=0;i<length;++i){value^=data[i];for(int bit=0;bit<8;++bit)value=(value>>1)^((value&1)?0xa001:0);}return value;}
inline void append16(std::string& out,uint16_t value){out.push_back(static_cast<char>(value&255));out.push_back(static_cast<char>(value>>8));}
inline uint16_t read16(const std::string& data,size_t index){return static_cast<unsigned char>(data[index])|(uint16_t(static_cast<unsigned char>(data[index+1]))<<8);}
inline std::string frame(uint8_t kind,uint16_t seq,const std::string& payload){
  if(payload.size()>64)throw std::invalid_argument("UART payload exceeds protocol limit");std::string body;body.push_back(1);body.push_back(static_cast<char>(kind));append16(body,seq);append16(body,static_cast<uint16_t>(payload.size()));body+=payload;
  auto checksum=crc(reinterpret_cast<const unsigned char*>(body.data()),body.size());std::string out="\xa5\x5a";out+=body;append16(out,checksum);return out;
}
inline std::string controls(uint16_t seq,const Values& values){std::string payload;for(float value:values){if(!std::isfinite(value))throw std::invalid_argument("Nonfinite UART target");uint32_t bits=std::bit_cast<uint32_t>(value);for(int b=0;b<4;++b)payload.push_back(static_cast<char>((bits>>(8*b))&255));}return frame(1,seq,payload);}
inline Values values(const std::string& packet,size_t offset=8){Values result;for(int i=0;i<4;++i){uint32_t bits=0;for(int b=0;b<4;++b)bits|=uint32_t(static_cast<unsigned char>(packet[offset+4*i+b]))<<(8*b);result[i]=std::bit_cast<float>(bits);}return result;}
struct Packet {uint8_t kind=0;uint16_t seq=0;Values values{};uint8_t ack=0;};
class Parser {
public:
  std::vector<Packet> feed(const std::string& data,double now){
    if(last_rx_&&now-*last_rx_>.1)buffer_.clear();if(!data.empty())last_rx_=now;buffer_+=data;
    std::vector<Packet> result;
    while(!buffer_.empty()){
      auto start=buffer_.find(std::string("\xa5\x5a",2));if(start==std::string::npos){buffer_=static_cast<unsigned char>(buffer_.back())==0xa5?std::string(1,char(0xa5)):"";break;}buffer_.erase(0,start);if(buffer_.size()<8)break;
      size_t size=read16(buffer_,6);if(size>64){buffer_.erase(0,1);continue;}if(buffer_.size()<size+10)break;
      if(crc(reinterpret_cast<const unsigned char*>(buffer_.data()+2),size+6)!=read16(buffer_,size+8)){buffer_.erase(0,1);continue;}
      auto packet=buffer_.substr(0,size+10);buffer_.erase(0,size+10);if(packet[2]!=1)continue;
      Packet out;out.kind=static_cast<uint8_t>(packet[3]);out.seq=read16(packet,4);
      if(out.kind==0x80&&size==1){out.ack=static_cast<uint8_t>(packet[8]);result.push_back(out);}
      else if(out.kind==0x81&&size==16){out.values=values(packet);if(std::all_of(out.values.begin(),out.values.end(),[](float v){return std::isfinite(v);}))result.push_back(out);}
    }
    if(buffer_.size()>8192)buffer_.clear();return result;
  }
private:std::string buffer_;std::optional<double> last_rx_;
};
class Session {
public:
  Parser parser;std::optional<uint16_t> sequence;std::optional<Packet> state;
  std::optional<double> state_time,ack_time;std::optional<uint8_t> last_ack;std::string pending,error;
  double sent_at=0,next_at=0;int retries=0;
  void fail(const std::string& why){error=why;pending.clear();sequence.reset();}
  void receive(const std::string& bytes,double now){for(const auto& packet:parser.feed(bytes,now)){
    if(packet.kind==0x81){state=packet;state_time=now;if(!sequence&&pending.empty())sequence=static_cast<uint16_t>(packet.seq+1);}
    else if(!pending.empty()&&sequence&&packet.seq==*sequence){last_ack=packet.ack;if(packet.ack<=1){pending.clear();sequence=static_cast<uint16_t>(*sequence+1);ack_time=now;}else fail("MCU ACK="+std::to_string(packet.ack));}
  }}
  std::string poll(const Values& target,double now){
    if(!error.empty()||!sequence)return {};if(!state_time||now-*state_time>.3){fail("MCU status timeout; reconnect and synchronize");return {};}
    if(!pending.empty()){if(now-sent_at<.05)return {};if(retries>=3){fail("ACK timeout after 3 retries; reconnect and synchronize");return {};}++retries;sent_at=now;return pending;}
    if(now<next_at)return {};pending=controls(*sequence,target);sent_at=now;next_at=now+.02;retries=0;return pending;
  }
};
struct Settings {
  std::string port;double forward=10,left=10,rot=1,max_translation=30,max_rot=6,yaw=0,max_speed=.5;
  explicit Settings(const fs::path& path){auto y=YAML::LoadFile(path.string());if(y["transport"].as<std::string>()!="navcar_uart1_v1"||y["baud"].as<int>()!=2000000)throw std::invalid_argument("Unsupported commissioned UART transport");port=y["serial_port"].as<std::string>();if(port.empty())throw std::invalid_argument("Serial port is required");
    forward=y["forward_units_per_mps"].as<double>();left=y["left_units_per_mps"].as<double>();rot=y["rot_units_per_radps"].as<double>();max_translation=y["max_translation_units"].as<double>();max_rot=y["max_rot_units"].as<double>();yaw=y["yaw_target_deg"].as<double>();
    for(double v:{forward,left,rot,max_translation,max_rot})if(!std::isfinite(v)||v<=0)throw std::invalid_argument("Invalid serial unit mapping");if(!std::isfinite(yaw))throw std::invalid_argument("Invalid yaw target");max_speed=std::min(.5,max_translation/std::max(forward,left));
  }
  Settings()=default;
};
class Controller {
public:
  explicit Controller(Settings settings={}):settings(settings){}
  Settings settings;Session session;bool armed=false,spin_enabled=false;
  std::optional<double> yaw;double strength=.1,updated=0,spin_rate=.1,spin_updated=0;
  std::string action="stop",owner,lease;uint64_t manual_epoch=0;std::optional<uint64_t> navigation_sequence;
  std::map<std::string,uint64_t> clients;Eigen::Vector2d navigation=Eigen::Vector2d::Zero();
  bool connected(double now) const{return session.state_time&&now-*session.state_time<.3;}
  bool spin_live(double now) const{return spin_enabled&&now-spin_updated<.2;}
  void receive(const std::string& bytes,double now){session.receive(bytes,now);if(!yaw&&session.state)yaw=session.state->values[3];}
  Json::Value state(double now) const {
    Json::Value s;s["connected"]=connected(now);s["error"]=session.error;s["active"]=armed&&now-updated<.2?action:"stop";s["control_started"]=armed;s["strength"]=strength;s["max_strength"]=settings.max_translation/std::max(settings.forward,settings.left);s["speed_unit"]="m/s";s["navigation_max_speed_mps"]=settings.max_speed;
    s["translation_scale"]["forward"]=settings.forward;s["translation_scale"]["left"]=settings.left;s["control_source"]=lease.empty()?"manual":"navigation";s["manual_epoch"]=Json::UInt64(manual_epoch);s["control_client"]=owner.empty()?Json::Value():Json::Value(owner);
    s["spin"]["enabled"]=spin_live(now);s["spin"]["rate"]=spin_rate;s["spin"]["max_rate"]=settings.max_rot/settings.rot;s["spin"]["unit"]="command";s["yaw_target_deg"]=yaw?Json::Value(*yaw):Json::Value();s["last_ack"]=session.last_ack?Json::Value(*session.last_ack):Json::Value();s["feedback_targets"]=Json::Value();
    if(session.state)for(float value:session.state->values)s["feedback_targets"].append(value);s["feedback_age_s"]=session.state_time?Json::Value(now-*session.state_time):Json::Value();return s;
  }
  std::string acquire(double now){
    if(!connected(now)||!session.error.empty()||!yaw)throw std::runtime_error("下位机未就绪");
    if(!lease.empty()||spin_live(now)||(action!="stop"&&now-updated<.2))throw std::runtime_error("控制器正在使用，请先停止当前控制");
    lease=random_token();navigation_sequence.reset();navigation.setZero();action="stop";spin_enabled=false;return lease;
  }
  void release(const std::string& value){if(!lease.empty()&&value==lease){lease.clear();navigation.setZero();action="stop";updated=0;}}
  void navigation_command(const Json::Value& data,double now){
    double x=finite_number(data,"forward"),y=finite_number(data,"left");auto seq=sequence_number(data);
    if(std::hypot(x,y)>settings.max_speed+1e-9)throw std::invalid_argument("导航速度超出范围");
    if(lease.empty()||!data["lease"].isString()||data["lease"].asString()!=lease)throw std::runtime_error("导航控制权已失效");
    if(!connected(now)||!session.error.empty())throw std::runtime_error("下位机未就绪");if(navigation_sequence&&seq<=*navigation_sequence)return;
    navigation_sequence=seq;spin_enabled=false;navigation<<x,y;action=x||y?"navigation":"stop";updated=now;armed=true;
  }
  void manual(const Json::Value& data,double now){
    if(!data["action"].isString())throw std::invalid_argument("需要方向");auto direction=data["action"].asString();
    if(!std::set<std::string>{"forward","backward","left","right","stop"}.contains(direction))throw std::invalid_argument("无效方向");
    double speed=finite_number(data,"strength");if(speed<=0||speed>settings.max_translation/std::max(settings.forward,settings.left))throw std::invalid_argument("目标速度超出范围");
    if(direction!="stop"&&data["speed_unit"].asString()!="m/s"&&(settings.forward!=1||settings.left!=1))throw std::invalid_argument("速度单位已更新，请刷新网页");
    bool spin=data.isMember("spin_enabled")?boolean(data,"spin_enabled"):false;double rate=data.isMember("spin_rate")?finite_number(data,"spin_rate"):spin_rate;validate_spin(spin,rate);
    if(!accept(data,now,direction!="stop"||spin,false))return;action=direction;strength=speed;updated=now;spin_enabled=spin;spin_rate=rate;spin_updated=now;armed=true;
  }
  void set_yaw(const Json::Value& data,double now){double angle=finite_number(data,"angle_deg");if(angle< -180||angle>180)throw std::invalid_argument("yaw目标角超出范围");if(!accept(data,now,true,true))return;action="stop";updated=0;spin_enabled=false;navigation.setZero();yaw=angle;armed=true;}
  void set_spin(const Json::Value& data,double now){bool enabled=boolean(data,"enabled");double rate=data.isMember("rate")?finite_number(data,"rate"):spin_rate;validate_spin(enabled,rate);Json::Value command=data;
    command["action"]=enabled&&owner==data["client"].asString()&&now-updated<.2&&action!="navigation"?action:"stop";command["strength"]=strength;command["speed_unit"]="m/s";command["spin_enabled"]=enabled;command["spin_rate"]=rate;manual(command,now);
  }
  Values desired(double now) const {
    Eigen::Vector2d velocity=Eigen::Vector2d::Zero();auto active=now-updated<.2?action:"stop";
    if(active=="navigation"&&!lease.empty())velocity=navigation;else if(active=="forward")velocity.x()=strength;else if(active=="backward")velocity.x()=-strength;else if(active=="left")velocity.y()=strength;else if(active=="right")velocity.y()=-strength;
    std::array<double,4> native{velocity.x()*settings.forward,-velocity.y()*settings.left,-(spin_live(now)?spin_rate:0.)*settings.rot,std::remainder(yaw.value_or(0),360.)};
    for(int i=0;i<3;++i)if(!std::isfinite(native[i])||std::abs(native[i])>(i==2?settings.max_rot:settings.max_translation))throw std::invalid_argument("Command exceeds commissioned native limits");Values out;std::transform(native.begin(),native.end(),out.begin(),[](double v){return static_cast<float>(v);});return out;
  }
private:
  static bool boolean(const Json::Value& data,const std::string& key){if(!data[key].isBool())throw std::invalid_argument("Boolean required: "+key);return data[key].asBool();}
  void validate_spin(bool enabled,double rate) const {if(!std::isfinite(rate)||std::abs(rate)>settings.max_rot/settings.rot||(enabled&&rate==0))throw std::invalid_argument("Invalid independent spin rate");}
  bool accept(const Json::Value& data,double now,bool wants_motion,bool takeover){
    if(!data["client"].isString()||data["client"].asString().empty()||data["client"].asString().size()>80)throw std::invalid_argument("无效控制会话");auto client=data["client"].asString();auto seq=sequence_number(data);
    if(wants_motion&&(!connected(now)||!session.error.empty()||!yaw))throw std::runtime_error("下位机未就绪");auto it=clients.find(client);if(it!=clients.end()&&seq<=it->second)return false;if(it==clients.end()&&clients.size()>=64)throw std::runtime_error("控制会话已满");
    bool active=(action!="stop"&&now-updated<.2)||spin_live(now);if(wants_motion&&!takeover&&owner!=client&&active&&lease.empty())throw std::runtime_error("另一个页面正在控制");owner=client;clients[client]=seq;lease.clear();++manual_epoch;return true;
  }
};
}
