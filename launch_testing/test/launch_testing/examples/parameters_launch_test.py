# Copyright 2019 Apex.AI, Inc.
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
import sys
import unittest

import ament_index_python

import launch
import launch.actions
import launch_testing
import launch_testing.util


@launch_testing.parametrize('arg_param', ['thing=On', 'thing=Off', 'flag1'])
def generate_test_description(arg_param, ready_fn):

    terminating_process = launch.actions.ExecuteProcess(
        cmd=[
            sys.executable,
            os.path.join(
                ament_index_python.get_package_prefix('launch_testing'),
                'lib/launch_testing',
                'terminating_proc',
            ),
            # Use the parameter passed to generate_test_description as an argument
            # to the terminating_proc
            '--{}'.format(arg_param),
        ]
    )

    return (
        launch.LaunchDescription([
            terminating_process,
            launch_testing.util.KeepAliveProc(),
            launch.actions.OpaqueFunction(function=lambda context: ready_fn())
        ]),
        {'dut_process': terminating_process}
    )


class TestProcessOutput(unittest.TestCase):

    # Note that 'arg_param' is automatically given to the test case, even though it was not
    # part of the test context.
    def test_process_outputs_expectd_value(self, proc_output, arg_param):
        proc_output.assertWaitFor('--' + arg_param, timeout=10)
