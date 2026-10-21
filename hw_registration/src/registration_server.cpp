// small_gicp quality assessment follows HWSentryNav26 (MIT); see THIRD_PARTY.md.
#include <Eigen/Eigenvalues>
#include <rclcpp/rclcpp.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>
#include <shaoshao_hw_interfaces/srv/register_cloud.hpp>

class RegistrationServer : public rclcpp::Node {
  using Cloud=small_gicp::PointCloud;
  using Tree=small_gicp::KdTree<Cloud>;
  using Service=shaoshao_hw_interfaces::srv::RegisterCloud;
  Cloud::Ptr map_;
  std::shared_ptr<Tree> tree_;
  rclcpp::Service<Service>::SharedPtr service_;
  double voxel_;
  int threads_;
  Cloud::Ptr prepare(const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud) {
    std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>> points;
    points.reserve(pcl_cloud.size());
    for(const auto& p:pcl_cloud) if(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z))
      points.emplace_back(p.x,p.y,p.z,1.);
    auto cloud=std::make_shared<Cloud>(points);
    if(!cloud->size())throw std::runtime_error("Empty cloud");
    cloud=small_gicp::voxelgrid_sampling_omp(*cloud,voxel_,threads_);
    if(cloud->size()<500)throw std::runtime_error("Fewer than 500 downsampled points");
    auto tree=std::make_shared<Tree>(cloud,small_gicp::KdTreeBuilderOMP(threads_));
    small_gicp::estimate_covariances_omp(*cloud,*tree,20,threads_);
    return cloud;
  }
  void align(const std::shared_ptr<Service::Request> req,std::shared_ptr<Service::Response> out) {
    out->normalized_error=out->condition_number=std::numeric_limits<double>::infinity();
    try {
      if(req->source.header.frame_id!="navcar_odom" || req->source.width*req->source.height>200000)
        throw std::runtime_error("Unexpected source frame/size");
      pcl::PointCloud<pcl::PointXYZ> pcl_source;pcl::fromROSMsg(req->source,pcl_source);
      if(pcl_source.size()<1000)throw std::runtime_error("Fewer than 1000 source points");
      auto source=prepare(pcl_source);
      const auto& p=req->initial_guess;
      Eigen::Quaterniond q(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z);
      if(!q.coeffs().allFinite()||std::abs(q.norm()-1.)>.01)throw std::runtime_error("Invalid initial rotation");
      Eigen::Isometry3d initial=Eigen::Isometry3d::Identity();initial.linear()=q.normalized().toRotationMatrix();
      initial.translation()<<p.position.x,p.position.y,p.position.z;
      if(!initial.matrix().allFinite())throw std::runtime_error("Invalid initial translation");
      small_gicp::Registration<small_gicp::GICPFactor,small_gicp::ParallelReductionOMP> registration;
      registration.reduction.num_threads=threads_;registration.optimizer.max_iterations=16;
      registration.criteria.translation_eps=.001;registration.criteria.rotation_eps=.001;
      registration.rejector.max_dist_sq=.25;
      const auto result=registration.align(*map_,*source,*tree_,initial);
      const Eigen::Matrix<double,6,6> h=.5*(result.H+result.H.transpose());
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6>> eigen(h);
      out->converged=result.converged&&result.T_target_source.matrix().allFinite()&&
        h.allFinite()&&eigen.info()==Eigen::Success;
      out->inliers=result.num_inliers;out->points=source->size();
      if(result.num_inliers)out->normalized_error=result.error/result.num_inliers;
      out->overlap=double(result.num_inliers)/source->size();
      if(out->converged){
        out->min_eigenvalue=eigen.eigenvalues().minCoeff();
        if(out->min_eigenvalue>0)out->condition_number=eigen.eigenvalues().maxCoeff()/out->min_eigenvalue;
      }
      const auto& t=result.T_target_source;Eigen::Quaterniond rot(t.rotation());
      out->transform.position.x=t.translation().x();out->transform.position.y=t.translation().y();out->transform.position.z=t.translation().z();
      out->transform.orientation.x=rot.x();out->transform.orientation.y=rot.y();
      out->transform.orientation.z=rot.z();out->transform.orientation.w=rot.w();
    }catch(const std::exception& e){out->converged=false;out->error=e.what();}
  }
public:
  RegistrationServer():Node("hw_registration") {
    auto path=declare_parameter<std::string>("map_path","");
    voxel_=declare_parameter<double>("voxel_m",.05);threads_=declare_parameter<int>("threads",2);
    if(voxel_<.01||voxel_>.2||threads_<1||threads_>16)throw std::runtime_error("Invalid GICP settings");
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if(pcl::io::loadPCDFile(path,cloud)<0)throw std::runtime_error("Cannot load map PCD");
    map_=prepare(cloud);tree_=std::make_shared<Tree>(map_,small_gicp::KdTreeBuilderOMP(threads_));
    service_=create_service<Service>("/hw/register",[this](std::shared_ptr<Service::Request> request,
      std::shared_ptr<Service::Response> response){align(request,response);});
    RCLCPP_INFO(get_logger(),"Registration ready, %zu map points",map_->size());
  }
};
int main(int argc,char** argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<RegistrationServer>());}
catch(const std::exception& e){std::cerr<<e.what()<<std::endl;rclcpp::shutdown();return 1;}rclcpp::shutdown();}
