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

import os
import subprocess

import ament_index_python


def test_ready_action_within_timeout_duration():
    testpath = os.path.join(
        ament_index_python.get_package_share_directory('launch_testing'),
        'examples',
        'ready_action_timeout_launch_test.py',
    )

    # the process takes 5sec to start which is within the
    # timeout duration of 20sec specified by ReadyToTest action.
    completed_process = subprocess.run(
        args=[
            'launch_test',
            '--timeout',
            '20.0',
            testpath
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    # success
    assert 0 == completed_process.returncode


# the process takes longer time to startup than the one
# specified by ReadyToTest action timeout duration.
def test_ready_action_exceed_timeout_duration():
    testpath = os.path.join(
        ament_index_python.get_package_share_directory('launch_testing'),
        'examples',
        'ready_action_timeout_launch_test.py',
    )

    # the process takes around 5 sec to start which exceeds the
    # 2 sec timeout duration specified by ReadyToTest action.
    completed_process = subprocess.run(
        args=[
            'launch_test',
            '--timeout',
            '2.0',
            testpath
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    # fail
    assert 1 == completed_process.returncode
    assert 'Timed out waiting for processes to start up' in \
           completed_process.stdout.decode()
