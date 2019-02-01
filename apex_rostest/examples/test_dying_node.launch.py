# Copyright 2018 Apex.AI, Inc.
# All rights reserved.

import time
import unittest

from launch import LaunchDescription
from launch_ros.actions import Node

import std_msgs.msg


def generate_test_description(ready_fn):
    # This is wrong - but here for test purposes.  We shouldn't signal ready until the node starts
    # Once we have some utilities built to enable waiting on nodes, this ready_fn should be
    # triggered as part of the LaunchDecription
    ready_fn()

    return LaunchDescription([
        Node(package='apex_rostest', node_executable='dying_node', output='screen')
    ])


class ExampleTest(unittest.TestCase):

    def test_a1(self):
        time.sleep(1.0)

    def test_that_kills_node(self):

        test_pub = self.node.create_publisher(
            msg_type=std_msgs.msg.String,
            topic="self_destruct"
        )

        time.sleep(1.0)
        msg = std_msgs.msg.String()
        msg.data = "kill the node under test"
        test_pub.publish(msg)

        time.sleep(1.0)

    def test_z1(self):
        time.sleep(1.0)

    def test_z2(self):
        time.sleep(1.0)

    def test_z3(self):
        time.sleep(1.0)
