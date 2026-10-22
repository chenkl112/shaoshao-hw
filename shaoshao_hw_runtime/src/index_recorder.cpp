#include "shaoshao_runtime/ros.hpp"
#include "shaoshao_runtime/index.hpp"
#include "shaoshao_runtime/pcd.hpp"
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace hw_runtime {
class Recorder:public rclcpp::Node {
public:
  Recorder():Node("hw_index_recorder"),config_(declare_parameter<std::string>("config_path",(runtime_path()/"shaoshao/web_panel/robot.yaml").string())),maps_(declare_parameter<std::string>("maps_dir","")),writer_(maps_/"keyframes",config_.index){
    auto runtime=fs::path(declare_parameter<std::string>("runtime_root",runtime_path().string()));auto lio=YAML::LoadFile((runtime/"mapping_runtime/fast_lio.yaml").string());offset_=yaml_point(lio["/**"]["ros__parameters"]["mapping"]["extrinsic_T"]);
    buffer_=std::make_shared<tf2_ros::Buffer>(get_clock());listener_=std::make_shared<tf2_ros::TransformListener>(*buffer_);
    subscription_=create_subscription<sensor_msgs::msg::PointCloud2>("/cloud_registered_body",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::PointCloud2::SharedPtr msg){if(!finalized_)latest_=msg;});
    timer_=create_wall_timer(std::chrono::milliseconds(100),std::bind(&Recorder::tick,this));
    service_=create_service<std_srvs::srv::Trigger>("/hw/finalize_index",[this](auto,auto response){
      try{tick();writer_.finalize(maps_/"site.pcd",read_pcd(maps_/"site.pcd"));finalized_=true;response->success=true;response->message="Indexed "+std::to_string(writer_.size())+" static keyframes";}
      catch(const std::exception& e){response->success=false;response->message=e.what();}
    });
  }
private:
  Config config_;fs::path maps_;IndexWriter writer_;Point offset_;bool finalized_=false;int64_t last_stamp_=0;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_;std::shared_ptr<tf2_ros::Buffer> buffer_;std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;rclcpp::TimerBase::SharedPtr timer_;rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  void tick(){
    if(!latest_||finalized_)return;int64_t stamp=int64_t(latest_->header.stamp.sec)*1000000000+latest_->header.stamp.nanosec;
    if(stamp<=last_stamp_||!fresh(now().seconds(),latest_->header.stamp,.5))return;
    try{auto tf=buffer_->lookupTransform("odom",latest_->header.frame_id,rclcpp::Time(latest_->header.stamp),rclcpp::Duration::from_seconds(.02));auto sensor=ros_transform(tf.transform);sensor.translation()+=sensor.linear()*offset_;auto cloud=decode(*latest_);for(auto& p:cloud)p-=offset_;writer_.add(cloud,sensor,stamp);last_stamp_=stamp;}
    catch(const std::exception& e){RCLCPP_DEBUG(get_logger(),"Index recorder waiting: %s",e.what());}
  }
};
}
int main(int argc,char** argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<hw_runtime::Recorder>());}catch(const std::exception& e){RCLCPP_ERROR(rclcpp::get_logger("hw_index_recorder"),"%s",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
