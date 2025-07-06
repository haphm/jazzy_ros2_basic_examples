/**
 * @file cpp_minimal_publisher.cpp
 * @author hapham (ha.pham@hamk.fi)
 * @brief 
 * @version 0.0.0
 * @date 2025-07-04
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "rclcpp/rclcpp.hpp" // ROS2 C++ client library
#include "std_msgs/msg/string.hpp" // Standard message type for strings

using namespace std::chrono_literals; // Handles time duration

class MinimalCppPublisher : public rclcpp::Node
{
public:
  MinimalCppPublisher() : Node("minimal_cpp_publisher"), count_(0)
  {
    publisher_ = create_publisher<std_msgs::msg::String>("/cpp_example_topic", 10);

    timer_ = create_wall_timer(500ms, std::bind(&MinimalCppPublisher::timerCallback, this));

    RCLCPP_INFO(get_logger(), "Publishing at 2Hz");
  }

  void timerCallback() 
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello! " + std::to_string(count_++);

    publisher_ -> publish(message);
  }

private:
  size_t count_; // Keeps track of the number of messages published
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_; // The publisher object
  rclcpp::TimerBase::SharedPtr timer_; // Timer
};

#ifndef TESTING_EXCLUDE_MAIN
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto minimal_cpp_publisher_node = std::make_shared<MinimalCppPublisher>();

  rclcpp::spin(minimal_cpp_publisher_node);

  rclcpp::shutdown();

  return 0;
}
#endif

