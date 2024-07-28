#include <rclcpp/rclcpp.hpp>
#include <cpp_calc/twice_node.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cpp_calc::TwiceNode>());
  rclcpp::shutdown();
  return 0;
}
