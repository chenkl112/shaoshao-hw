#include "hw_omni_controller/relay.hpp"
#include "hw_omni_controller/ros_samples.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <shaoshao_hw_interfaces/msg/nav_command.hpp>
#include <shaoshao_hw_interfaces/msg/navigation_session.hpp>
#include <shaoshao_hw_interfaces/msg/localization_status.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>
#include <sstream>

class CommandRelayNode : public rclcpp::Node {
public:
  CommandRelayNode():Node("hw_command_relay"){
    relay_.max_speed=declare_parameter("max_speed_mps",.5);
    relay_.deceleration=declare_parameter("deceleration_mps2",.3);
    relay_.jerk=declare_parameter("jerk_mps3",1.);
    relay_.radius=declare_parameter("collision_radius_m",.317658);
    relay_.feedback=hw_omni::FeedbackGate(declare_parameter("feedback_timeout_s",.15));
    if(!std::isfinite(relay_.max_speed)||relay_.max_speed<=0||relay_.max_speed>.5||
       !std::isfinite(relay_.deceleration)||relay_.deceleration<=0||!std::isfinite(relay_.jerk)||
       relay_.jerk<=0||!std::isfinite(relay_.radius)||relay_.radius<=0)
      throw std::invalid_argument("Invalid command relay limits");
    buffer_=std::make_shared<tf2_ros::Buffer>(get_clock());listener_=std::make_shared<tf2_ros::TransformListener>(*buffer_);
    command_pub_=create_publisher<shaoshao_hw_interfaces::msg::NavCommand>("/hw/nav_command",1);
    status_pub_=create_publisher<std_msgs::msg::String>("/hw/command/status",1);
    session_sub_=create_subscription<shaoshao_hw_interfaces::msg::NavigationSession>("/hw/navigation_session",1,
      [this](shaoshao_hw_interfaces::msg::NavigationSession::SharedPtr msg){
        if(msg->header.frame_id=="navcar_center")relay_.update_session(
          {hw_omni::seconds(msg->header.stamp),msg->lease,msg->authority_epoch,msg->enabled},now().seconds());
      });
    feedback_sub_=create_subscription<shaoshao_hw_interfaces::msg::ChassisFeedback>("/hw/chassis_feedback",10,
      [this](shaoshao_hw_interfaces::msg::ChassisFeedback::SharedPtr msg){relay_.update_feedback(hw_omni::chassis_sample(*msg),now().seconds());});
    velocity_sub_=create_subscription<geometry_msgs::msg::TwistStamped>("/navcar/cmd_vel_auto",1,
      [this](geometry_msgs::msg::TwistStamped::SharedPtr msg){
        if(msg->header.frame_id!="navcar_center"||msg->twist.angular.z!=0||msg->twist.linear.z!=0||
           msg->twist.angular.x!=0||msg->twist.angular.y!=0){relay_.raw_stamp=0;return;}
        double stamp=hw_omni::seconds(msg->header.stamp),age=now().seconds()-stamp;
        if(age<-.05||age>.2||stamp<=relay_.raw_stamp)return;
        relay_.raw_stamp=stamp;relay_.raw=hw_omni::U(msg->twist.linear.x,msg->twist.linear.y);
      });
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/navcar/odom",10,
      [this](nav_msgs::msg::Odometry::SharedPtr msg){
        double stamp=hw_omni::seconds(msg->header.stamp),age=now().seconds()-stamp;
        if(msg->child_frame_id=="navcar_center"&&age>=-.05&&age<=.15&&stamp>odom_stamp_&&std::isfinite(msg->twist.twist.angular.z)){
          odom_stamp_=stamp;radar_omega_=msg->twist.twist.angular.z;
        }
      });
    localization_sub_=create_subscription<shaoshao_hw_interfaces::msg::LocalizationStatus>("/hw/localization/state",10,
      [this](shaoshao_hw_interfaces::msg::LocalizationStatus::SharedPtr msg){
        double stamp=hw_omni::seconds(msg->header.stamp),age=now().seconds()-stamp;
        if(stamp<=relay_.localization_stamp)return;
        relay_.localized=msg->header.frame_id=="map"&&age>=-.05&&age<=.5&&msg->ready&&
          (msg->state==1||msg->state==2)&&std::isfinite(msg->speed_limit_mps)&&msg->speed_limit_mps>0&&msg->speed_limit_mps<=.5;
        if(age>=-.05&&age<=.5&&msg->header.frame_id=="map")relay_.localization_stamp=stamp;
        relay_.localization_speed=msg->speed_limit_mps;
      });
    prediction_sub_=create_subscription<shaoshao_hw_interfaces::msg::PredictedObstacles>("/hw/predicted_obstacles",1,
      [this](shaoshao_hw_interfaces::msg::PredictedObstacles::SharedPtr msg){
        double stamp=hw_omni::seconds(msg->header.stamp);if(stamp<=relay_.prediction_stamp)return;
        try{relay_.obstacles=hw_omni::read_predictions(*msg,now().seconds());relay_.prediction_stamp=stamp;relay_.prediction_valid=true;}
        catch(const std::exception&){relay_.prediction_valid=false;}
      });
    grid_sub_=create_subscription<nav_msgs::msg::OccupancyGrid>("/local_costmap/costmap",rclcpp::QoS(1).transient_local(),
      std::bind(&CommandRelayNode::grid,this,std::placeholders::_1));
    timer_=create_wall_timer(std::chrono::milliseconds(20),std::bind(&CommandRelayNode::tick,this));
  }
private:
  hw_omni::CommandRelay relay_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Publisher<shaoshao_hw_interfaces::msg::NavCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<shaoshao_hw_interfaces::msg::NavigationSession>::SharedPtr session_sub_;
  rclcpp::Subscription<shaoshao_hw_interfaces::msg::ChassisFeedback>::SharedPtr feedback_sub_;
  rclcpp::Subscription<shaoshao_hw_interfaces::msg::LocalizationStatus>::SharedPtr localization_sub_;
  rclcpp::Subscription<shaoshao_hw_interfaces::msg::PredictedObstacles>::SharedPtr prediction_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double radar_omega_=0,odom_stamp_=0;
  void grid(nav_msgs::msg::OccupancyGrid::SharedPtr msg){
    double stamp=hw_omni::seconds(msg->header.stamp),age=now().seconds()-stamp;
    if(stamp<=relay_.grid_stamp)return;
    const auto& info=msg->info;const auto& p=info.origin.position;const auto& q=info.origin.orientation;
    bool valid=msg->header.frame_id=="map"&&age>=-.05&&age<=.5&&info.width&&info.height&&
      size_t(info.width)*info.height<=1000000&&msg->data.size()==size_t(info.width)*info.height&&
      std::isfinite(info.resolution)&&info.resolution>=.01&&info.resolution<=.1&&
      std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(q.w)&&std::isfinite(q.z)&&
      std::abs(q.x)<1e-6&&std::abs(q.y)<1e-6&&std::abs(q.z)<1e-6&&std::abs(std::abs(q.w)-1)<1e-6;
    if(!valid){relay_.grid.cells.clear();return;}
    hw_omni::Grid next;next.width=static_cast<int>(info.width);next.height=static_cast<int>(info.height);
    next.origin=hw_omni::U(p.x,p.y);next.resolution=info.resolution;next.cells.reserve(msg->data.size());
    for(int value:msg->data){
      if(value<-1||value>100){relay_.grid.cells.clear();return;}
      next.cells.push_back(value<0?255:value==100?254:value==99?253:static_cast<unsigned char>(std::round(value*252./98)));
    }
    relay_.grid=std::move(next);relay_.grid_stamp=stamp;
  }
  void tick(){
    auto current=now();
    try{
      auto pose=buffer_->lookupTransform("map","navcar_center",tf2::TimePointZero);
      double stamp=hw_omni::seconds(pose.header.stamp);
      if(current.seconds()-stamp>=-.05&&current.seconds()-stamp<=.15&&
         current.seconds()-odom_stamp_>=-.05&&current.seconds()-odom_stamp_<=.15&&relay_.feedback.sample()){
        relay_.position=hw_omni::U(pose.transform.translation.x,pose.transform.translation.y);
        double yaw=tf2::getYaw(pose.transform.rotation),age=std::max(0.0,current.seconds()-stamp);
        const auto& measured=*relay_.feedback.sample();
        relay_.feedback_yaw=yaw+radar_omega_*(measured.stamp-stamp);
        double c=std::cos(relay_.feedback_yaw),s=std::sin(relay_.feedback_yaw);
        relay_.position+=age*hw_omni::U(c*measured.vx-s*measured.vy,s*measured.vx+c*measured.vy);
        relay_.yaw=yaw+radar_omega_*age;relay_.pose_stamp=std::min(stamp,odom_stamp_);
      }
    }catch(const tf2::TransformException&){}
    auto output=relay_.step(current.seconds());
    if(!output.lease.empty()){
      shaoshao_hw_interfaces::msg::NavCommand command;command.header.frame_id="navcar_center";command.header.stamp=current;
      command.lease=output.lease;command.seq=output.sequence;command.twist.linear.x=output.velocity.x();command.twist.linear.y=output.velocity.y();
      command_pub_->publish(command);
    }
    std_msgs::msg::String status;std::ostringstream text;
    text<<"{\"stamp_ns\":"<<current.nanoseconds()<<",\"sent_xy\":["<<output.velocity.x()<<","<<output.velocity.y()<<"],\"moving_allowed\":"
      <<(output.moving?"true":"false")<<",\"reason\":\""<<output.reason<<"\"}";
    status.data=text.str();status_pub_->publish(status);
  }
};
int main(int argc,char** argv){
  rclcpp::init(argc,argv);
  try{rclcpp::spin(std::make_shared<CommandRelayNode>());}catch(const std::exception& e){RCLCPP_ERROR(rclcpp::get_logger("hw_command_relay"),"%s",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
