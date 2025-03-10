#include <rclcpp/rclcpp.hpp>

namespace utils
{
class RateCounter {
public:
  RateCounter(const rclcpp::Clock::SharedPtr& clk_ptr) : clk_ptr_(clk_ptr), prev_call_(clk_ptr_->now()), avg_dt_(0.1){};

  double update_rate() {

    const rclcpp::Time     cur_call = clk_ptr_->now();
    const rclcpp::Duration cur_dur  = cur_call - prev_call_;

    const double cur_dt = cur_dur.seconds();

    avg_dt_ = 0.90 * avg_dt_ + 0.1 * cur_dt;

    prev_call_ = cur_call;

    if (avg_dt_ > 0) {
      return 1.0 / avg_dt_;
    } else {
      return 0.0;
    }
  }

private:
  rclcpp::Clock::SharedPtr clk_ptr_;
  rclcpp::Time             prev_call_;
  double                   avg_dt_;
};

}  // namespace utils
