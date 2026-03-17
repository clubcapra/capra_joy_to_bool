#include "capra_joy_to_bool/joy_to_bool_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FlippersTeleop>());
    rclcpp::shutdown();
    return 0;
}
