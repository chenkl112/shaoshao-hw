#include <cmath>
#include <nav2_mppi_controller/critic_function.hpp>
#include <pluginlib/class_list_macros.hpp>
namespace mppi::critics {
class ShaoShaoNormCritic : public CriticFunction {
  float max_speed_=.5f;
public:
  void initialize() override {
    auto get=parameters_handler_->getParamGetter(name_);
    get(max_speed_,"max_speed_mps",.5f);
    if(max_speed_<=0||max_speed_>.5)throw std::runtime_error("Invalid coupled speed bound");
  }
  void score(CriticData& data) override {
    if(!enabled_)return;
    size_t valid=0;
    for(size_t i=0;i<data.state.cvx.shape(0);++i){
      bool legal=true;
      for(size_t j=0;j<data.state.cvx.shape(1);++j){
        if(std::hypot(data.state.cvx(i,j),data.state.cvy(i,j))>max_speed_+1e-5f ||
           std::abs(data.state.cwz(i,j))>1e-5f){legal=false;break;}
      }
      if(!legal)data.costs(i)+=1e9f;else ++valid;
    }
    if(!valid){
      data.fail_flag=true;
      RCLCPP_WARN_THROTTLE(logger_,*parent_.lock()->get_clock(),2000,"All MPPI candidates exceed coupled speed/rotation envelope");
    }
  }
};
}
PLUGINLIB_EXPORT_CLASS(mppi::critics::ShaoShaoNormCritic,mppi::critics::CriticFunction)
