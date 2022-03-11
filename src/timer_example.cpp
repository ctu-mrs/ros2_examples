#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float64.hpp>

using namespace std::chrono_literals;

namespace ros2_examples
{

  /* class TimerExample //{ */

  class TimerExample : public rclcpp::Node
  {
  public:
    TimerExample(rclcpp::NodeOptions options);

  private:
    // | ------------------------- timers ------------------------- |

    rclcpp::CallbackGroup::SharedPtr cb_grp_;

    rclcpp::TimerBase::SharedPtr timer_1_;
    rclcpp::TimerBase::SharedPtr timer_2_;
    rclcpp::TimerBase::SharedPtr timer_3_;

    void callback_timer1();
    void callback_timer2();
    void callback_timer3();

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_timer_1_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_timer_2_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_timer_3_;

    // | -------------- helper class to count a rate -------------- |
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

  };

  //}

  /* TimerExample() //{ */

  TimerExample::TimerExample(rclcpp::NodeOptions options) : Node("timer_example", options)
  {

    RCLCPP_INFO(get_logger(), "[TimerExample]: initializing");

    // | -------------------------- timer ------------------------- |

    // create separate callback group. Default group in Node class is CallbackGroupType::MutuallyExclusive
    // (https://github.com/ros2/rclcpp/blob/9c62c1c9463cd2accee57fe157125950df50f957/rclcpp/src/rclcpp/node_interfaces/node_base.cpp#L151) explanation of
    // CallbackGroupTypes: https://roscon.ros.org/2014/wp-content/uploads/2014/07/ROSCON-2014-Why-you-want-to-use-ROS-2.pdf
    cb_grp_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // Uncomment the following line to use a Reentrant callback group - the slow callback will no longer block the others.
    /* cb_grp_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant); */

    // 100 Hz time
    timer_1_ = create_wall_timer(std::chrono::duration<double>(1.0 / 100.0), std::bind(&TimerExample::callback_timer1, this), cb_grp_);

    // 10 Hz timer
    timer_2_ = create_wall_timer(std::chrono::duration<double>(1.0 / 10.0), std::bind(&TimerExample::callback_timer2, this), cb_grp_);

    // 1 Hz timer with 800ms work to do, it will block the other timers from being executed because if it is in the same MutuallyExclusive callback group.
    // It will not block if it is in a Reentrant callback group or a different callback group than the other callbacks.
    timer_3_ = create_wall_timer(std::chrono::duration<double>(1.0 / 1.0), std::bind(&TimerExample::callback_timer3, this), cb_grp_);

    publisher_timer_1_ = create_publisher<std_msgs::msg::Float64>("timer_1", 10);
    publisher_timer_2_ = create_publisher<std_msgs::msg::Float64>("timer_2", 10);
    publisher_timer_3_ = create_publisher<std_msgs::msg::Float64>("timer_3", 10);

    // | --------------------- finish the init -------------------- |

    RCLCPP_INFO(get_logger(), "[TimerExample]: initialized");
  }

  //}

  // | ------------------------ callbacks ----------------------- |

  /* callbackTimer1() //{ */

  void TimerExample::callback_timer1()
  {
    static RateCounter cntr(get_clock(), 100);
    const double rate = cntr.update_rate();

    publisher_timer_1_->publish(std_msgs::msg::Float64().set__data(rate));
    RCLCPP_INFO_STREAM(get_logger(), "[TimerExample]: 100 Hz Timer spinning at rate " << rate << "Hz");
  }

  //}

  /* callbackTimer2() //{ */

  void TimerExample::callback_timer2()
  {
    static RateCounter cntr(get_clock(), 30);
    const double rate = cntr.update_rate();

    publisher_timer_2_->publish(std_msgs::msg::Float64().set__data(rate));
    RCLCPP_INFO_STREAM(get_logger(), "[TimerExample]: 10 Hz Timer spinning at rate " << rate << "Hz");
  }

  //}

  /* callbackTimer3() //{ */

  void TimerExample::callback_timer3()
  {
    static RateCounter cntr(get_clock(), 10);
    const double rate = cntr.update_rate();

    publisher_timer_3_->publish(std_msgs::msg::Float64().set__data(rate));
    RCLCPP_INFO_STREAM(get_logger(), "[TimerExample]: 1 Hz timer spinning at rate " << rate << "Hz, starting work");
    rclcpp::sleep_for(std::chrono::milliseconds(800));
    RCLCPP_INFO(get_logger(), "[TimerExample]: 1 Hz timer stopping work");
  }

  //}

}  // namespace ros2_examples

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_examples::TimerExample)
