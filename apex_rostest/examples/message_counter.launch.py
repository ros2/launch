# Copyright 2018 Apex.AI, Inc.
# All rights reserved.

import unittest
import uuid

from launch import LaunchDescription
from launch_ros.actions import Node
import rclpy

from apex_rostest import post_shutdown_test

import std_msgs.msg
import test_msgs.srv


test_uuid = str(uuid.uuid4())


def generate_test_description(ready_fn):
    # This is wrong - but here for test purposes.  We shouldn't signal ready until the node starts
    # Once we have some utilities built to enable waiting on nodes, this ready_fn should be
    # triggered as part of the LaunchDecription
    ready_fn()

    return LaunchDescription([
        Node(package='apex_rostest', node_executable='message_counter', output='screen')
    ])


class ExampleTest(unittest.TestCase):

    def test_that_will_pass(self):
        self.assertIsNotNone(object())

    def test_that_talks_to_node(self):
        # This test will read the message counter's msg count, attempt to increment it, and then
        # verify that the count goes up

        msg_count_client = self.node.create_client(
            srv_type=test_msgs.srv.Primitives,
            srv_name="/get_message_count",
        )

        test_pub = self.node.create_publisher(
            msg_type=std_msgs.msg.String,
            topic="msgs"
        )

        # See how many messages the message counter node has seen so far:
        self.assertTrue(
            msg_count_client.wait_for_service(timeout_sec=5.0),
            "Timed out waiting for service"
        )

        def get_msg_count():
            future = msg_count_client.call_async(test_msgs.srv.Primitives.Request())
            rclpy.spin_until_future_complete(self.node, future)
            return future.result().int32_value

        initial_msg_count = get_msg_count()

        msg = std_msgs.msg.String()
        msg.data = test_uuid
        test_pub.publish(msg)

        import time
        time.sleep(1.0)  # message_counter lacks synchronizatoin

        final_msg_count = get_msg_count()
        self.assertEquals(final_msg_count, initial_msg_count + 1)


@post_shutdown_test()
class PostShutdownTests(unittest.TestCase):

    def test_good_exit_code(self):
        for info in self.proc_info:
            self.assertEquals(
                info.returncode,
                0,
                "Non-zero exit code for process {}".format(info.process_name)
            )

    def test_node_stdout(self):
        # Check that we see "Count is 1" somewhere in stdout
        self.assertTrue(
            any(
                map(lambda x: b"Count is 1" in x.text, self.proc_output),
            )
        )

    def test_msg_output(self):
        # Check that the UUID we sent to the node appears in stdout
        self.assertTrue(
            any(
                map(lambda x: test_uuid.encode('ascii') in x.text, self.proc_output),
            )
        )
