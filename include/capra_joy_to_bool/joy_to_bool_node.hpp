#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>

class FlippersTeleop : public rclcpp::Node
{
public:
  explicit FlippersTeleop(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

  bool latched_ = false;

  int button1_index_;
  int button2_index_;
  bool latch_mode_ = false;
  std::string joy_topic_ = "~/joy";
  std::string flippers_topic_ = "~/bool";

  // Track previous button states for edge detection
  int prev_button1_ = 0;
  int prev_button2_ = 0;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
};