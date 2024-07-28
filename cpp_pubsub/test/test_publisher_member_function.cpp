// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

class TestTalker : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<MinimalPublisher>();
    subscription_ = node_->create_subscription<std_msgs::msg::String>(
      "topic", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
        received_messages_.push_back(msg->data);
      });

    // Create a spinner
    executor.add_node(node_);

    // Run the executor in a separate thread
    executor_thread = std::thread(
      [&]() {
        executor.spin();
      });
  }

  void TearDown() override
  {
    // Stop the executor
    executor.cancel();

    // Join the executor thread
    executor_thread.join();
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::vector<std::string> received_messages_;

  rclcpp::executors::SingleThreadedExecutor executor;
  std::thread executor_thread;
};

TEST_F(TestTalker, PublishMessage)
{
  // Wait for a period of time
  constexpr int spin_duration_sec = 2;  // Adjust this value as needed
  std::this_thread::sleep_for(std::chrono::seconds(spin_duration_sec));


  // Check if any messages were received
  ASSERT_FALSE(received_messages_.empty());

  // Verify the content of the received messages
  for (const auto & message : received_messages_) {
    EXPECT_EQ(message.substr(0, 12), "Hello, world");
  }
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  rclcpp::init(argc, argv);

  auto ret = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return ret;
}
