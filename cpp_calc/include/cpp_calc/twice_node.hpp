#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

namespace cpp_calc
{
using Int32 = std_msgs::msg::Int32;

class TwiceNode : public rclcpp::Node
{
public:
  TwiceNode();

private:
  rclcpp::Subscription<Int32>::SharedPtr sub_;
  rclcpp::Publisher<Int32>::SharedPtr pub_;
};
}  // namespace cpp_calc
