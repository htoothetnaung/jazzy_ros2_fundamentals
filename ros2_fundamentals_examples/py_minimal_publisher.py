#! /usr/bin/env python3

"""
Description:
    This ROS2 node periodically publishes a message to a topic.


----------------------------
Publishing Topics:
    The channel containing the "Hello World" message.
    /py_example_topic - std_msgs/String

Subscription Topics:
    None
----------------------------
Author: Htoo Thet Naung
Date: 2025-05-13
"""
import rclpy  # Import the ros2 client library for python
from rclpy.node import Node

from std_msgs.msg import String  # Import the message type to be published


class MinimalPyPublisher(Node):

    def __init__(self):

        # Initialize the node with the name 'py_minimal_publisher'
        super().__init__('py_minimal_publisher')

        # Create a publisher that will publish to the topic '/py_example_topic'
        self.publisher_1 = self.create_publisher(String, '/py_example_topic', 10)

        # Create a timer with a period of 0.5 seconds to trigger publishing of message
        self.timer = self.create_timer(0.5, self.timer_callback)

        # Initialize a counter variable for message content
        self.i = 0

    def timer_callback(self):
        """Callback function executed periodically by the timer."""

        msg = String()  # Create a new String message
        msg.data = 'Hello World: %d' % self.i

        # publish the message you created abive to topic
        self.publisher_1.publish(msg)

        # Log the message indicating that it has been published
        self.get_logger().info('Publishing: "%s"' % msg.data)

        # Increment the counter for the next message
        self.i += 1


def main(args=None):
    """
    Main function to start the ROS2 node.

    Args:
        args (list, optional): Command line arguments (default: None).
    """

    # Initialize Ros2 python client library
    rclpy.init(args=args)

    # Create an instance of the MinimalPyPublisher class
    minimal_py_publisher = MinimalPyPublisher()

    rclpy.spin(minimal_py_publisher)  # Keep the node running until interrupted

    # Destroy the node explicitly
    minimal_py_publisher.destroy_node()  # optional tho

    # Shutdown the ROS2 python client library
    rclpy.shutdown()


if __name__ == '__main__':
    main() # Execute the main function when the script is run directly
