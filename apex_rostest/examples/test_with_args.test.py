# Copyright 2018 Apex.AI, Inc.
# All rights reserved.

import unittest

from launch import LaunchDescription
import launch.actions
import launch.substitutions
import launch_ros.actions

import apex_rostest
import apex_rostest.asserts
import apex_rostest.util


dut_node = launch_ros.actions.Node(
    package='apex_rostest',
    node_executable='terminating_node',
    arguments=[launch.substitutions.LaunchConfiguration('dut_node_arg')],
)


def generate_test_description(ready_fn):

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'dut_node_arg',
            default_value=['default'],
            description='Passed to the terminating node',
        ),
        dut_node,
        apex_rostest.util.EmptyNode(),
        launch.actions.OpaqueFunction(function=lambda context: ready_fn())
    ])


class TestTerminatingNodeStops(unittest.TestCase):

    def test_node_terminates(self):
        self.proc_info.assertWaitForShutdown(process=dut_node, timeout=10)


@apex_rostest.post_shutdown_test()
class TestNodeOutput(unittest.TestCase):

    def test_ran_with_arg(self):
        self.assertNotIn(
            'default',
            dut_node.process_details['cmd'],
            "Try running: apex_rostest test_with_args.test.py dut_node_arg:=arg"
        )

    def test_arg_printed_in_output(self):
        apex_rostest.asserts.assertInStdout(
            self.proc_output,
            self.test_args['dut_node_arg'],
            dut_node
        )

    def test_default_not_printed(self):
        with self.assertRaises(AssertionError):
            apex_rostest.asserts.assertInStdout(
                self.proc_output,
                "default",
                dut_node
            )
