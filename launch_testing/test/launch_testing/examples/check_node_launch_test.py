import time
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from rclpy.node import Node


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        launch.actions.TimerAction(
            period=5.0,
            actions=[
                launch_ros.actions.Node(
                    package='demo_nodes_cpp',
                    executable='talker',
                    name='demo_node_1',
                )
            ]),
        launch_testing.actions.ReadyToTest()
    ])


class TestFixture(unittest.TestCase):

    def test_node_start(self, proc_output):
        rclpy.init()
        node = MakeTestNode('test_node')
        assert node.wait_for_node('demo_node_1', 8.0), 'Node not found !'
        rclpy.shutdown()


class MakeTestNode(Node):

    def __init__(self, name='test_node'):
        super().__init__(name)

    def wait_for_node(self, node_name, timeout=8.0):
        start = time.time()
        flag = False
        print('Waiting for node...')
        while time.time() - start < timeout and not flag:
            flag = node_name in self.get_node_names()
            time.sleep(0.1)

        return flag
