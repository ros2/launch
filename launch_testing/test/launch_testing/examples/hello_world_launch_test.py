import unittest

import launch
import launch.actions
import launch_testing.actions
import pytest


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=['echo', 'hello_world']
        ),
        launch_testing.actions.ReadyToTest()
    ])


class TestHelloWorldProcess(unittest.TestCase):

    def test_read_stdout(self, proc_output):
        proc_output.assertWaitFor('hello_world', timeout=10, stream='stdout')


@launch_testing.post_shutdown_test()
class TestHelloWorldShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
