#include <chrono>

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "happymoon_control.h"
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  rclcpp::NodeOptions happymoon_quad_control_options;
  happymoon_quad_control_options.arguments(
      {"--ros-args"});
  auto happymoon_quad_control = std::make_shared<happymoon_control::HappyMoonControl>(happymoon_quad_control_options);
  exec.add_node(happymoon_quad_control);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}