/**
 * @file cpp_minimal_subscriber.cpp
 * @author hapham (ha.pham@hamk.fi)
 * @brief 
 * @version 0.0.0
 * @date 2025-07-04
 * 
 * ------
 * Subsciption Topics:
 *  String message
 *  /cpp_example_topic - std_msgs/String
 * ------
 * Publishing Topics:
 *  None
 * 
 */

#include "rclcpp/rclcpp.hpp" // ROS2 C++ Client Library
#include "std_msgs/msg/string.hpp" // Handling String messages

using std::placeholders::_1; // Placeholder for callback function

class MinimalCppSubscriber : public rclcpp::Node
{
public:
  MinimalCppSubscriber() : Node("minimal_cpp_subscriber")
  {
    subscriber_ = create_subscription<std_msgs::msg::String>
    ("/cpp_example_topic", 10, std::bind(&MinimalCppSubscriber::topicCallback, this, _1));
  }

  void topicCallback(const std_msgs::msg::String & msg) const
  {
    RCLCPP_INFO_STREAM(get_logger(), "Received message: " << msg.data.c_str());
  }
private:
  // Member variables
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto minimal_cpp_subscriber_node = std::make_shared<MinimalCppSubscriber>();

  rclcpp::spin(minimal_cpp_subscriber_node);

  rclcpp::shutdown();

  return 0;
}

