# Copyright 2023 Metro Robots
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

"""Tests the utility function shutdown_on_process_exit"""

import sys
import time

from launch import LaunchDescription
from launch import LaunchService
from launch.actions.execute_process import ExecuteProcess
from launch.utilities.shutdown_on_process_exit import shutdown_on_process_exit


def test_shutdown_on_process_exit():
    def generate_launch_description():
        process_action = ExecuteProcess(
            cmd=[sys.executable, '-c', 'import time; time.sleep(5)'],
        )
        process_action2 = ExecuteProcess(
            cmd=[sys.executable, '-c', 'import time; time.sleep(55)'])
        ld = LaunchDescription()
        ld.add_action(process_action)
        ld.add_action(process_action2)
        ld.add_action(shutdown_on_process_exit(process_action))
        return ld

    ls = LaunchService(noninteractive=True)
    ls.include_launch_description(generate_launch_description())
    start_t = time.time()
    assert 0 == ls.run()
    end_t = time.time()
    duration = end_t - start_t
    assert duration > 5
    assert duration < 10

    ls = LaunchService()  # interactive
    ls.include_launch_description(generate_launch_description())
    start_t = time.time()
    assert 0 == ls.run()
    end_t = time.time()
    duration = end_t - start_t
    assert duration > 5
    assert duration < 10
