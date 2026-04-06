#include "capra_joy_to_bool/joy_to_bool_node.hpp"

JoyToBool::JoyToBool(const rclcpp::NodeOptions &options)
    : Node("joy_to_bool", options)
{
    // Declare parameters
    button1_index_ = this->declare_parameter<int>("button1_index", 0);
    button2_index_ = this->declare_parameter<int>("button2_index", -1);
    use_stamped_ = this->declare_parameter<bool>("use_stamped", use_stamped_);

    latch_mode_ = (button2_index_ != -1);
    // Publisher
    if (use_stamped_) {
        pub_stamped_ = this->create_publisher<capra_control_msgs::msg::BoolStamped>(bool_topic_, 1);
    } else {
        pub_ = this->create_publisher<std_msgs::msg::Bool>(bool_topic_, 1);
    }
    // Subscriber
    sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        joy_topic_,
        10,
        std::bind(&JoyToBool::joyCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
        get_logger(),
        "Listening to button %d on topic '%s', publishing to '%s'",
        button1_index_,
        joy_topic_.c_str(),
        bool_topic_.c_str());
}

void JoyToBool::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    bool out;
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
                msg->buttons.size());
            out = false;
        }
        else
        {
            out = (msg->buttons[button1_index_] != 0);
        }
    }
    else
    {
        bool button1_valid = (button1_index_ >= 0 &&
                              button1_index_ < static_cast<int>(msg->buttons.size()));
        bool button2_valid = (button2_index_ >= 0 &&
                              button2_index_ < static_cast<int>(msg->buttons.size()));

        if (!button1_valid)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Button index %d out of range (buttons size: %zu)",
                button1_index_,
                msg->buttons.size());
        }

        if (!button2_valid)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Button index %d out of range (buttons size: %zu)",
                button2_index_,
                msg->buttons.size());
        }

        // Latch on rising edge of button1 only
        if (button1_valid)
        {
            int cur = msg->buttons[button1_index_];
            if (cur && !prev_button1_)
            {
                latched_ = true;
            }
            prev_button1_ = cur;
        }

        // Unlatch on rising edge of button2 only
        if (button2_valid)
        {
            int cur = msg->buttons[button2_index_];
            if (cur && !prev_button2_)
            {
                latched_ = false;
            }
            prev_button2_ = cur;
        }

        out= latched_;
    }

    if (use_stamped_) {
        auto bool_msg = capra_control_msgs::msg::BoolStamped();
        bool_msg.header = msg->header;
        bool_msg.data = out;
        pub_stamped_->publish(bool_msg);
    } else {
        auto bool_msg = std_msgs::msg::Bool();
        bool_msg.data = out;
        pub_->publish(bool_msg);
    }


}