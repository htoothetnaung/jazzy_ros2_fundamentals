/**
 * @file cpp_minimal_publisher.cpp
 * @author Htoo Thet Naung (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2025-05-17
 *
 * @copyright Copyright (c) 2025
 *
 */


#include "rclcpp/rclcpp.hpp" // Ros2 C++ client library
#include "std_msgs/msg/string.hpp" // String message type

using namespace std::chrono_literals; // For using time literals

class MinimalCppPublisher : public rclcpp::Node
{
public:
  MinimalCppPublisher() : Node("minimal_cpp_publisher"), count_(0)
  {
    publisher_ = create_publisher<std_msgs::msg::String>(
      "/cpp_example_topic", 10); // Create a publisher on the topic "/cpp_example_topic"

    timer_ = create_wall_timer(500ms,
    std::bind(&MinimalCppPublisher::timerCallback, this)); // Create a timer that calls the callback every 500ms

    RCLCPP_INFO(get_logger(), "Publishing at 2Hz");
  }

  void timerCallback(){
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);

    publisher_ -> publish(message);
  }

private:
  size_t count_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto minimal_cpp_publisher_node = std::make_shared<MinimalCppPublisher>();
  rclcpp::spin(minimal_cpp_publisher_node); // Keep the node alive and processing callbacks
  rclcpp::shutdown(); // Shutdown the node

  return 0;
}