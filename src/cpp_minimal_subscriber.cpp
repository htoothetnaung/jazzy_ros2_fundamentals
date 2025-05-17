/**
 * @file cpp_minimal_subscriber.cpp
 * @author Htoo Thet Naung (you@domain.com)
 * @brief Demonstration of subscribing to a topic in ROS2 using C++
 * @version 0.1
 * @date 2025-05-17
 *
 * @copyright Copyright (c) 2025
 *
 *
 * ------------------------------
 * Subscription Topics:
 *  String message
 *  /cpp_example_topic - std_msgs/String
 *
 * ------------------------------
 *
 * Publishing Topics:
 *  None
 *
 *
 */
#include "rclcpp/rclcpp.hpp" // Ros2 C++ client library
#include "std_msgs/msg/string.hpp" // Handling String messages


using std::placeholders::_1; // For using placeholders in callbacks

class MinimalCppSubscriber : public rclcpp::Node
{
  public:
    MinimalCppSubscriber() : Node("minimal_cpp_subscriber")
    {
      subscriber_ = create_subscription<std_msgs::msg::String>(
        "/cpp_example_topic", 10,
        std::bind(&MinimalCppSubscriber::topicCallback, this, _1) // Create a subscription to the topic "/cpp_example_topic"
      );
    };

    void topicCallback(const std_msgs::msg::String &msg) const {
      RCLCPP_INFO_STREAM(get_logger(), "I heard: " << msg.data); // Log the received message
    }


  private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;

};

int main(int argc, char* argv[]){
  rclcpp::init(argc, argv);
  auto minimal_cpp_subscriber_node = std::make_shared<MinimalCppSubscriber>();

  rclcpp::spin(minimal_cpp_subscriber_node);
  rclcpp::shutdown();

  return 0;
}


