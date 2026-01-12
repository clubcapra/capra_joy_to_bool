#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>

class FlippersTeleop : public rclcpp::Node
{
public:
  FlippersTeleop()
  : Node("joy_to_bool")
  {
    // Declare parameters
    button1_index_ = this->declare_parameter<int>("button1_index", 0);
    button2_index_ = this->declare_parameter<int>("button2_index", -1);

    if (button2_index_ != -1) {
      latch_mode_ = true;
    }

    // Publisher
    pub_ = this->create_publisher<std_msgs::msg::Bool>(flippers_topic_, 10);

    // Subscriber
    sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_,
      10,
      std::bind(&FlippersTeleop::joyCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(
      get_logger(),
      "Listening to button %d on topic '%s', publishing to '%s'",
      button1_index_,
      joy_topic_.c_str(),
      flippers_topic_.c_str()
    );
  }

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    std_msgs::msg::Bool out;
    if (!latch_mode_) 
    {
      if (button1_index_ < 0 || button1_index_ >= static_cast<int>(msg->buttons.size()))
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Button index %d out of range (buttons size: %zu)",
          button1_index_,
          msg->buttons.size()
        );
        out.data = false;
      }
      else
      {
        out.data = (msg->buttons[button1_index_] != 0);
      }
    }
    else
    {
      if (button1_index_ < 0 || button1_index_ >= static_cast<int>(msg->buttons.size()))
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Button index %d out of range (buttons size: %zu)",
          button1_index_,
          msg->buttons.size()
        );
        latched_ = false;
      }
      else
      {
        if (msg->buttons[button1_index_]) {
          latched_ = true;
        }
      }
      if (button1_index_ == 0 || button1_index_ >= static_cast<int>(msg->buttons.size()))
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Button index %d out of range (buttons size: %zu)",
          button1_index_,
          msg->buttons.size()
        );
        latched_ = false;
      }
      else
      {
        if (msg->buttons[button2_index_]) {
          latched_ = false;
        }
      }
      out.data = latched_;
    }


    pub_->publish(out);
  }

  bool latched_ = false;

  int button1_index_;
  int button2_index_;
  bool latch_mode_;
  std::string joy_topic_ = "~/joy";
  std::string flippers_topic_ = "~/bool";

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlippersTeleop>());
  rclcpp::shutdown();
  return 0;
}
