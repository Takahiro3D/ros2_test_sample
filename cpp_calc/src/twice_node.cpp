#include "cpp_calc/twice_node.hpp"
#include "cpp_calc/twice.hpp"

namespace cpp_calc
{
TwiceNode::TwiceNode()
: rclcpp::Node("twice", "")
{
  pub_ = this->create_publisher<Int32>("dst", 10);
  sub_ = this->create_subscription<Int32>(
    "src", 10, [this](const Int32::ConstSharedPtr src) -> void {
      auto output = Int32();
      output.data = do_twice(src->data);
      this->pub_->publish(output);
    }
  );
}
}  // namespace cpp_calc
