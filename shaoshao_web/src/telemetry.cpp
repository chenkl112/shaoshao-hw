#include "shaoshao_web/backend.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <hw_omni_controller/cloud.hpp>
#include <functional>

namespace shaoshao_web {
class Telemetry : public rclcpp::Node {
public:
  explicit Telemetry(std::shared_ptr<Backend> backend):Node("shaoshao_web_telemetry"),backend_(std::move(backend)){
    metadata_pub_=create_publisher<std_msgs::msg::String>("/hw/web/cloud_status",1);
    cloud_sub_=create_subscription<sensor_msgs::msg::PointCloud2>("/cloud_registered",rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg){
        if(!backend_->active()||Clock::now()-last_cloud_<std::chrono::milliseconds(180))return;
        double age=now().seconds()-(msg->header.stamp.sec+msg->header.stamp.nanosec*1e-9);
        if(age<-.05||age>.5)return;
        try{
          std::array<hw_obstacles::FloatField,3> fields;std::array<bool,3> found{};
          const std::array<std::string,3> names{"x","y","z"};
          for(const auto& field:msg->fields)for(int i=0;i<3;++i)if(field.name==names[i]&&field.count==1){fields[i]={field.offset,field.datatype};found[i]=true;}
          if(!std::all_of(found.begin(),found.end(),[](bool value){return value;}))return;
          auto decoded=hw_obstacles::decode_cloud(msg->data,msg->width,msg->height,msg->point_step,msg->row_step,msg->is_bigendian,fields);
          size_t step=std::max(size_t(1),(decoded.size()+5999)/6000);std::vector<std::array<float,3>> points;
          for(size_t i=0;i<decoded.size();i+=step)points.push_back({static_cast<float>(decoded[i].x()),static_cast<float>(decoded[i].y()),static_cast<float>(decoded[i].z())});
          auto count=points.size();backend_->update_cloud(std::move(points),msg->header.frame_id,"mid360_registered");
          metadata(msg->header,count,"mid360_registered");last_cloud_=Clock::now();
        }catch(const std::exception& e){RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,"%s",e.what());}
      });
    raw_sub_=create_subscription<livox_ros_driver2::msg::CustomMsg>("/livox/lidar",rclcpp::SensorDataQoS(),
      [this](livox_ros_driver2::msg::CustomMsg::SharedPtr msg){
        if(backend_->active()||Clock::now()-last_raw_<std::chrono::milliseconds(180))return;
        double age=now().seconds()-(msg->header.stamp.sec+msg->header.stamp.nanosec*1e-9);
        if(age<-.05||age>.5||msg->points.size()>200000)return;
        size_t step=std::max(size_t(1),(msg->points.size()+5999)/6000);std::vector<std::array<float,3>> points;
        for(size_t i=0;i<msg->points.size();i+=step){const auto& p=msg->points[i];if(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z))points.push_back({p.x,p.y,p.z});}
        auto count=points.size();backend_->update_cloud(std::move(points),msg->header.frame_id,"mid360_raw");
        metadata(msg->header,count,"mid360_raw");last_raw_=Clock::now();
      });
  }
private:
  std::shared_ptr<Backend> backend_;Clock::time_point last_raw_{},last_cloud_{};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr raw_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metadata_pub_;
  void metadata(const std_msgs::msg::Header& header,size_t count,const std::string& source){
    Json::Value value;value["stamp_ns"]=Json::Int64(static_cast<int64_t>(header.stamp.sec)*1000000000+header.stamp.nanosec);
    value["count"]=static_cast<unsigned>(count);value["frame"]=header.frame_id;value["source"]=source;
    std_msgs::msg::String msg;msg.data=json(value);metadata_pub_->publish(msg);
  }
};
std::function<void()> start_telemetry(std::shared_ptr<Backend> backend){
  rclcpp::InitOptions options;rclcpp::init(0,nullptr,options,rclcpp::SignalHandlerOptions::None);
  auto node=std::make_shared<Telemetry>(std::move(backend));
  auto executor=std::make_shared<rclcpp::executors::SingleThreadedExecutor>();executor->add_node(node);
  auto thread=std::make_shared<std::thread>([executor]{executor->spin();});
  return [node,executor,thread]{executor->cancel();if(thread->joinable())thread->join();executor->remove_node(node);rclcpp::shutdown();};
}
}
