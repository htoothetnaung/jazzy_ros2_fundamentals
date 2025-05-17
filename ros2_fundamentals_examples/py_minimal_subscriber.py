#! /usr/bin/env python3

"""
Description:
    This ROS2 node subscribes to a topic and prints the received messages.
----------------------------
Publishing Topics:
    None
-----------------------------
Subscription Topics:
    The channel containing the "Hello World" message.
    /py_example_topic - std_msgs/String

----------------------------
Author: Htoo Thet Naung
Date: 2025-05-13
"""

import rclpy  # Import the ros2 client library for python
from rclpy.node import Node
from std_msgs.msg import String  # Import the message type to be published


class MinimalPySubscriber(Node):
    def __init__(self):

        super().__init__('py_minimal_subscriber')  # Initialize the node with the name 'py_minimal_subscriber'

        self.subscriber_1 = self.create_subscription(
            String,
            '/py_example_topic',  # Topic to subscribe to
            self.listener_callback,  # Callback function to handle incoming messages
            10  # Queue size for incoming messages
        )

    def listener_callback(self, msg):

        self.get_logger().info(f'I heard: "{msg.data}"')  # Log the received message


def main(args=None):

    rclpy.init(args=args)  # Initialize the ROS2 client library
    minimal_py_subscriber = MinimalPySubscriber()  # Create an instance of the subscriber node

    rclpy.spin(minimal_py_subscriber)  # Keep the node running and processing incoming messages

    minimal_py_subscriber.destroy_node()  # Clean up the node when done
    rclpy.shutdown()  # Shutdown the ROS2 client library


if __name__ == '__main__':
    main()
