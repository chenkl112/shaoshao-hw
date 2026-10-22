#pragma once
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>

namespace hw_omni {
struct ChassisSample {
  double stamp=0, vx=0, vy=0, omega=0, gimbal_yaw=0;
  uint64_t sequence=0, authority_epoch=0;
  bool control_allowed=false;
  std::string frame="navcar_center";
};
enum class SampleUpdate { accepted, replayed, invalid };

class FeedbackGate {
public:
  explicit FeedbackGate(double timeout=0.15):timeout_(timeout){
    if(!std::isfinite(timeout)||timeout<=0||timeout>0.2)
      throw std::invalid_argument("Invalid chassis feedback timeout");
  }
  SampleUpdate update(const ChassisSample& value,double now){
    if(!std::isfinite(now)||!std::isfinite(value.stamp)||!std::isfinite(value.vx)||
       !std::isfinite(value.vy)||!std::isfinite(value.omega)||!std::isfinite(value.gimbal_yaw)||
       value.frame!="navcar_center"||now-value.stamp<-.05||now-value.stamp>timeout_||
       std::hypot(value.vx,value.vy)>3||std::abs(value.omega)>30){
      invalid_=true;return SampleUpdate::invalid;
    }
    if(sample_&&(value.authority_epoch<sample_->authority_epoch||
        (value.authority_epoch==sample_->authority_epoch&&
         (value.sequence<=sample_->sequence||value.stamp<=sample_->stamp))))
      return SampleUpdate::replayed;
    sample_=value;invalid_=false;return SampleUpdate::accepted;
  }
  bool ready(double now) const {
    return sample_&&!invalid_&&sample_->control_allowed&&std::isfinite(now)&&
      now-sample_->stamp>=-.05&&now-sample_->stamp<=timeout_;
  }
  const std::optional<ChassisSample>& sample() const{return sample_;}
private:
  double timeout_;
  bool invalid_=false;
  std::optional<ChassisSample> sample_;
};
}
