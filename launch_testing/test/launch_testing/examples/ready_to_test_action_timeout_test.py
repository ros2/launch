# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import unittest

import launch
from launch.actions import TimerAction
import launch_testing
from launch_testing.actions import ReadyToTest

import pytest


# The ReadyToTest action waits for 15 second by default for the processes to
# start, if processes take more time an error is thrown. We use decorator here
# to provide timeout duration of 20 second so that processes that take longer than
# 15 seconds can start up.

@pytest.mark.launch_test
@launch_testing.ready_to_test_action_timeout(20)
def generate_test_description():
    # takes 15 sec for the TimerAction process to start
    return launch.LaunchDescription([
        launch_testing.util.KeepAliveProc(),
        TimerAction(period=15.0, actions=[ReadyToTest()]),
    ])


# the process should exit cleanly now since the process takes 15 second
# to start and timeout duration of ReadyToTest action is 20 seconds.
@launch_testing.post_shutdown_test()
class TestHelloWorldShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check if the processes exited normally."""
        launch_testing.asserts.assertExitCodes(proc_info)
