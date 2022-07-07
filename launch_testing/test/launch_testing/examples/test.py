# Copyright 2022 Apex.AI, Inc.
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

import launch
import launch_testing
from launch.actions import TimerAction
from launch_testing.actions import ReadyToTest

import pytest


@pytest.mark.launch_test
@launch_testing.ready_to_test_action_timeout(10.0)
def generate_test_description():
    # takes 5 sec for the TimerAction process to start
    return launch.LaunchDescription([
        launch_testing.util.KeepAliveProc(),
        TimerAction(period=5.0, actions=[ReadyToTest()]),
    ])
