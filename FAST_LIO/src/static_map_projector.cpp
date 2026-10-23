#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <std_srvs/srv/trigger.hpp>
#include "static_map_projection.hpp"
#include "static_map_export.hpp"

class StaticMapProjector:public rclcpp::Node {
    StaticMapProjection projection_;
    pcl::PointCloud<pcl::PointXYZI> map_;
    std_msgs::msg::Header header_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_,terrain_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double resolution_;
    bool dirty_=false;
    std::string map_path_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr export_;
public:
    StaticMapProjector():Node("static_map_projector") {
        resolution_=declare_parameter<double>("resolution",.05);
        map_path_=declare_parameter<std::string>("map_path","");
        if(!std::isfinite(resolution_)||resolution_<.01||resolution_>1.)
            throw std::invalid_argument("Static projected map resolution must be in [0.01, 1.0] m");
        pub_=create_publisher<nav_msgs::msg::OccupancyGrid>("/projected_map",rclcpp::QoS(1).transient_local().reliable());
        map_sub_=create_subscription<sensor_msgs::msg::PointCloud2>("/Laser_map",rclcpp::QoS(1).reliable(),
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){
                if(msg->header.frame_id!="odom") {RCLCPP_ERROR(get_logger(),"Static map must be in odom frame");return;}
                pcl::fromROSMsg(*msg,map_);header_=msg->header;dirty_=true;
            });
        terrain_sub_=create_subscription<sensor_msgs::msg::PointCloud2>("/terrain_map_ext",rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){
                if(msg->header.frame_id!="odom")return;
                pcl::PointCloud<pcl::PointXYZI> terrain;pcl::fromROSMsg(*msg,terrain);
                projection_.terrain(terrain);dirty_=true;
            });
        timer_=create_wall_timer(std::chrono::seconds(1),[this](){publish();});
        export_=create_service<std_srvs::srv::Trigger>("/hw/export_grid",
            [this](std_srvs::srv::Trigger::Request::SharedPtr,std_srvs::srv::Trigger::Response::SharedPtr response){
                try{
                    pcl::PointCloud<pcl::PointXYZI> saved;
                    if(map_path_.empty()||pcl::io::loadPCDFile(map_path_,saved)<0)throw std::runtime_error("Cannot read saved PCD for projection");
                    StaticMapProjection::Grid grid;grid.resolution=resolution_;
                    if(!projection_.project(saved,grid))throw std::runtime_error("Saved PCD/floor data cannot produce a 2D map");
                    export_static_grid(std::filesystem::path(map_path_).parent_path(),grid);
                    response->success=true;response->message="Projected saved PCD to PNG/YAML";
                }catch(const std::exception& e){response->success=false;response->message=e.what();}
            });
    }
    void publish() {
        if(!dirty_)return;
        StaticMapProjection::Grid grid;
        grid.resolution=resolution_;
        try {
            if(!projection_.project(map_,grid)) {
                // Clear the latched map as well when the last static voxel disappears.
                if(map_.empty()) {
                    nav_msgs::msg::OccupancyGrid msg;msg.header=header_;msg.header.frame_id="map";
                    msg.info.resolution=resolution_;msg.info.width=msg.info.height=1;
                    msg.info.origin.orientation.w=1.;msg.data={-1};pub_->publish(msg);
                }
                return;
            }
        }catch(const std::exception &e){RCLCPP_ERROR(get_logger(),"%s",e.what());return;}
        nav_msgs::msg::OccupancyGrid msg;msg.header=header_;msg.header.frame_id="map";
        msg.info.map_load_time=header_.stamp;msg.info.resolution=grid.resolution;
        msg.info.width=grid.width;msg.info.height=grid.height;
        msg.info.origin.position.x=grid.x;msg.info.origin.position.y=grid.y;
        msg.info.origin.orientation.w=1.;msg.data=std::move(grid.data);
        pub_->publish(msg);dirty_=false;
    }
};
int main(int argc,char **argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<StaticMapProjector>());rclcpp::shutdown();}
