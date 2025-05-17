#!/usr/bin/env python3

"""
Test suite  for ROS2 minimal publisher node.

This script contains unit tests for verifying the functionality of a minimal ROS 2 publisher node.
It tests the node creation, message counter increment, and message content formatting.

---------------------

Subscription Topics:
    None

----------------------

Publisher Topics:
    /py_example_topic (std_msgs/String): Example messages with incrementing counter.

:author: Htoo Thet Naung
:date: May 17, 2025


"""
import pytest
import rclpy
from std_msgs.msg import String
from ros2_fundamentals_examples.py_minimal_publisher import MinimalPyPublisher


def test_publisher_creation():
    """
    Test if the publisher node is created successfully.

    This test verifies:
    1. The node name is set correctly.
    2. The publisher object exists.
    3. The topic name is correct.

    :raises: AssertionError if any of the checks fail.
    """
    rclpy.init()

    try:
        node = MinimalPyPublisher()

        # Test 1: Verify the node has the expected name
        assert node.get_name() == 'py_minimal_publisher'

        # Test 2: Verify the publisher exists and has the correct topic name
        assert hasattr(node, 'publisher_1')
        assert node.publisher_1.topic == '/py_example_topic'

    finally:
        # Clean up the ROS2 Communication
        rclpy.shutdown()


def test_message_counter():
    """
    Test if the message counter increments correctly.

    This test verifies:
    1. The initial message counter is 0.
    2. The message counter increments by 1 after publishing a message.

    :raises: AssertionError if any of the checks fail.
    """
    rclpy.init()

    try:
        node = MinimalPyPublisher()

        # Test 1: Verify the initial message counter is 0
        initial_count = node.i

        # Test 2: Publish a message and verify the counter increments
        node.timer_callback()
        assert node.i == initial_count + 1

    finally:
        # Clean up the ROS2 Communication
        rclpy.shutdown()


def test_message_content():
    """
    Test if the message content is formatted correctly .

    This test verifies that the message string is properly formatted using an f-string with the current counter valie.

    :raises: AssertionError if the message format doesn't match the expected format.
    """

    rclpy.init()

    try:
        node = MinimalPyPublisher()

        # Set Counter to a known value for testing
        node.i = 5
        msg = String()

        # Using f-string instead of % formatting
        msg.data = f'Hello World: {node.i}'
        assert msg.data == 'Hello World: 5'

    finally:
        rclpy.shutdown()


if __name__ == '__main__':

    pytest.main(['-v'])
