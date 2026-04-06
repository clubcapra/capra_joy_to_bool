#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <capra_control_msgs/msg/bool_stamped.hpp>

class JoyToBool : public rclcpp::Node
{
public:
  explicit JoyToBool(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

  bool latched_ = false;

  int button1_index_;
  int button2_index_;
  bool latch_mode_ = false;
  std::string joy_topic_ = "~/joy";
  std::string bool_topic_ = "~/bool";
  bool use_stamped_ = false;

  // Track previous button states for edge detection
  int prev_button1_ = 0;
  int prev_button2_ = 0;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
  rclcpp::Publisher<capra_control_msgs::msg::BoolStamped>::SharedPtr pub_stamped_;
};