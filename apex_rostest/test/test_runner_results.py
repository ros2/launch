# Copyright 2018 Apex.AI, Inc.
# All rights reserved.

import imp
import mock

from launch import LaunchDescription
from launch_ros.actions import Node

from apex_rostest import ApexRunner


# Run tests on nodes that die early with an exit code and make sure the results returned
# indicate failure
def test_dut_with_exception():

    def generate_test_description(ready_fn):
        ready_fn()

        return LaunchDescription([
            Node(package='apex_rostest',
                 node_executable='exception_node',
                 output='screen')
        ])

    with mock.patch('apex_rostest.ApexRunner._run_test'):
        runner = ApexRunner(
            gen_launch_description_fn=generate_test_description,
            test_module=None
        )

        pre_result, post_result = runner.run()

        assert not pre_result.wasSuccessful()
        assert not post_result.wasSuccessful()


# Run some known good tests to check the nominal-good test path
def test_nominally_good_dut():

    def generate_test_description(ready_fn):
        ready_fn()

        return LaunchDescription([
            Node(package='apex_rostest',
                 node_executable='message_counter',
                 output='screen')
        ])

    # This string is the test code that the ApexRunner executes
    test_code = """
import unittest
from apex_rostest import post_shutdown_test

class PreTest(unittest.TestCase):
    def test_pre_ok(self):
        pass

@post_shutdown_test()
class PostTest(unittest.TestCase):
    def test_post_ok(self):
        pass
    """
    # Blah blah blah, get the code into a module so we can trick ApexRunner into running it
    module = imp.new_module("test_module")
    exec(test_code, module.__dict__)

    # Here's the actual 'test' part of the test:
    runner = ApexRunner(
        gen_launch_description_fn=generate_test_description,
        test_module=module
    )

    pre_result, post_result = runner.run()

    assert pre_result.wasSuccessful()

    assert pre_result.wasSuccessful()
