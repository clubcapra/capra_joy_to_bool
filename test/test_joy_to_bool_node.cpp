#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>

#include "capra_joy_to_bool/joy_to_bool_node.hpp"

using namespace std::chrono_literals;

class JoyToBoolNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    helper_node_ = std::make_shared<rclcpp::Node>("joy_to_bool_test_helper");
  }

  void createIo()
  {
    joy_pub_ = helper_node_->create_publisher<sensor_msgs::msg::Joy>("/joy_to_bool/joy", 10);
    bool_sub_ = helper_node_->create_subscription<std_msgs::msg::Bool>(
      "/joy_to_bool/bool", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        last_bool_msg_ = *msg;
        got_bool_msg_ = true;
      });
  }

  bool publishAndWait(const std::vector<int32_t> & buttons, bool expected)
  {
    sensor_msgs::msg::Joy joy_msg;
    joy_msg.buttons = buttons;

    got_bool_msg_ = false;
    joy_pub_->publish(joy_msg);

    auto start = std::chrono::steady_clock::now();
    while ((std::chrono::steady_clock::now() - start) < 2s) {
      executor_.spin_some();
      if (got_bool_msg_) {
        return last_bool_msg_.data == expected;
      }
      std::this_thread::sleep_for(10ms);
    }

    return false;
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<rclcpp::Node> helper_node_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bool_sub_;
  std_msgs::msg::Bool last_bool_msg_;
  bool got_bool_msg_{false};
};

TEST_F(JoyToBoolNodeTest, momentaryModePublishesButtonState)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("button1_index", 5);

  auto node = std::make_shared<JoyToBool>(options);
  createIo();

  executor_.add_node(node);
  executor_.add_node(helper_node_);

  EXPECT_TRUE(publishAndWait(std::vector<int32_t>(16, 0), false));

  auto pressed = std::vector<int32_t>(16, 0);
  pressed[5] = 1;
  EXPECT_TRUE(publishAndWait(pressed, true));

  EXPECT_TRUE(publishAndWait(std::vector<int32_t>(16, 0), false));

  executor_.remove_node(helper_node_);
  executor_.remove_node(node);
}

TEST_F(JoyToBoolNodeTest, latchModeSetsTrueWithButton1AndFalseWithButton2)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("button1_index", 14);
  options.append_parameter_override("button2_index", 15);

  auto node = std::make_shared<JoyToBool>(options);
  createIo();

  executor_.add_node(node);
  executor_.add_node(helper_node_);

  EXPECT_TRUE(publishAndWait(std::vector<int32_t>(16, 0), false));

  auto button1_pressed = std::vector<int32_t>(16, 0);
  button1_pressed[14] = 1;
  EXPECT_TRUE(publishAndWait(button1_pressed, true));

  EXPECT_TRUE(publishAndWait(std::vector<int32_t>(16, 0), true));

  auto button2_pressed = std::vector<int32_t>(16, 0);
  button2_pressed[15] = 1;
  EXPECT_TRUE(publishAndWait(button2_pressed, false));

  executor_.remove_node(helper_node_);
  executor_.remove_node(node);
}

TEST_F(JoyToBoolNodeTest, latchModeSetsTrueWithButton1AndFalseWithButton2AndKeepsFalse)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("button1_index", 14);
  options.append_parameter_override("button2_index", 15);

  auto node = std::make_shared<JoyToBool>(options);
  createIo();

  executor_.add_node(node);
  executor_.add_node(helper_node_);

  auto off = std::vector<int32_t>(16, 0);
  auto button1_pressed = std::vector<int32_t>(16, 0);
  button1_pressed[14] = 1;
  auto button2_pressed = std::vector<int32_t>(16, 0);
  button2_pressed[15] = 1;

  EXPECT_TRUE(publishAndWait(off, false));
  EXPECT_TRUE(publishAndWait(off, false));
  EXPECT_TRUE(publishAndWait(off, false));
  
  EXPECT_TRUE(publishAndWait(button1_pressed, true));
  EXPECT_TRUE(publishAndWait(off, true));
  EXPECT_TRUE(publishAndWait(off, true));
  EXPECT_TRUE(publishAndWait(off, true));


  EXPECT_TRUE(publishAndWait(button2_pressed, false));
  EXPECT_TRUE(publishAndWait(off, false));
  EXPECT_TRUE(publishAndWait(off, false));
  EXPECT_TRUE(publishAndWait(off, false));

  executor_.remove_node(helper_node_);
  executor_.remove_node(node);
}


