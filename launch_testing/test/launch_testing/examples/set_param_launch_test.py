import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
import pytest
from rcl_interfaces.msg import Parameter, ParameterType
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.node import Node


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='demo_nodes_cpp',
            executable='parameter_blackboard',
            name='demo_node_1',
        ),
        launch_testing.actions.ReadyToTest()
    ])


class TestFixture(unittest.TestCase):

    def test_set_parameter(self, proc_output):
        rclpy.init()
        node = MakeTestNode('test_node')
        response = node.set_parameter(state=True)
        assert response.successful, 'Could not set parameter!'
        rclpy.shutdown()


class MakeTestNode(Node):

    def __init__(self, name='test_node'):
        super().__init__(name)

    def set_parameter(self, state=True, timeout=5.0):
        parameter = Parameter()
        parameter.name = 'demo_parameter_1'
        parameter.value.bool_value = state
        parameter.value.type = ParameterType.PARAMETER_BOOL
        parameters = [parameter]

        client = self.create_client(SetParameters, 'demo_node_1/set_parameters')
        ready = client.wait_for_service(timeout_sec=5.0)
        if not ready:
            raise RuntimeError('Wait for service timed out')

        request = SetParameters.Request()
        request.parameters = parameters
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response is None:
            e = future.exception()
            raise RuntimeError(
                f"Exception while calling service of node 'demo_node_1': {e}")
        return response.results[0]
