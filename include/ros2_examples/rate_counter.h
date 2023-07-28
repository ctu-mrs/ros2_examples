#include <rclcpp/rclcpp.hpp>

namespace utils
{
  class RateCounter
  {
  public:
    RateCounter(const rclcpp::Clock::SharedPtr& clk_ptr, const unsigned window_len)
      : clk_ptr_(clk_ptr), prev_call_(clk_ptr_->now()), first_update_(true), avg_rate_(0), avg_rate_weight_(0), max_weight_(window_len-1) {};

    double update_rate()
    {
      // ignore the first update as it's probably going to be called right after construction
      if (first_update_)
      {
        first_update_ = false;
        return 0;
      }

      const rclcpp::Time cur_call = clk_ptr_->now();;
      const rclcpp::Duration cur_dur = cur_call - prev_call_;
      const double cur_rate = 1.0/cur_dur.seconds();

      std::scoped_lock lck(mtx_);
      avg_rate_ = (avg_rate_*avg_rate_weight_ + cur_rate)/(avg_rate_weight_+1);
      // clamp the max weight so that only the last N samples are taken into account
      avg_rate_weight_ = std::min(avg_rate_weight_+1, max_weight_);
      prev_call_ = cur_call;
      return avg_rate_;
    }
  private:
    std::mutex mtx_;
    rclcpp::Clock::SharedPtr clk_ptr_;
    rclcpp::Time prev_call_;
    bool first_update_;
    double avg_rate_;
    unsigned avg_rate_weight_;
    const unsigned max_weight_;
  };
}
