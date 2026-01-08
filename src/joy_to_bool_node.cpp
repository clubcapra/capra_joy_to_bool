#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>

class JoyToBool : public rclcpp::Node
{
public:
  JoyToBool()
  : Node("joy_to_bool")
  {
    // Declare parameters
    button_index_ = this->declare_parameter<int>("button_index", 0);

    // Publisher
    pub_ = this->create_publisher<std_msgs::msg::Bool>(bool_topic_, 10);

    // Subscriber
    sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_,
      10,
      std::bind(&JoyToBool::joyCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(
      get_logger(),
      "Listening to button %d on topic '%s', publishing to '%s'",
      button_index_,
      joy_topic_.c_str(),
      bool_topic_.c_str()
    );
  }

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    std_msgs::msg::Bool out;

    if (button_index_ < 0 || button_index_ >= static_cast<int>(msg->buttons.size()))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Button index %d out of range (buttons size: %zu)",
        button_index_,
        msg->buttons.size()
      );
      out.data = false;
    }
    else
    {
      out.data = (msg->buttons[button_index_] != 0);
    }

    pub_->publish(out);
  }

  int button_index_;
  std::string joy_topic_ = "~/joy";
  std::string bool_topic_ = "~/bool";

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyToBool>());
  rclcpp::shutdown();
  return 0;
}
