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
import unittest

import ament_index_python
import launch
import launch.actions
import launch.substitutions

import apex_launchtest
import apex_launchtest.util


dut_process = launch.actions.ExecuteProcess(
    cmd=[
        os.path.join(
            ament_index_python.get_package_prefix('apex_launchtest'),
            'lib/apex_launchtest',
            'terminating_proc',
        ),

        # Arguments
        launch.substitutions.LaunchConfiguration('dut_arg')
    ],
)


def generate_test_description(ready_fn):

    return launch.LaunchDescription([

        # This argument can be passed into the test, and can be discovered by running
        # apex_launchtest --show-args
        launch.actions.DeclareLaunchArgument(
            'dut_arg',
            default_value=['default'],
            description='Passed to the terminating process',
        ),

        dut_process,

        # In tests where all of the procs under tests terminate themselves, it's necessary
        # to add a dummy process not under test to keep the launch alive.  apex_launchtest
        # provides a simple launch action that does this:
        apex_launchtest.util.KeepAliveProc(),

        launch.actions.OpaqueFunction(function=lambda context: ready_fn())
    ])


class TestTerminatingProcessStops(unittest.TestCase):

    def test_proc_terminates(self):
        self.proc_info.assertWaitForShutdown(process=dut_process, timeout=10)


@apex_launchtest.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_ran_with_arg(self):
        self.assertNotIn(
            'default',
            dut_process.process_details['cmd'],
            "Try running: apex_launchtest test_with_args.test.py dut_arg:=arg"
        )

    def test_arg_printed_in_output(self):
        apex_launchtest.asserts.assertInStdout(
            self.proc_output,
            self.test_args['dut_arg'],
            dut_process
        )

    def test_default_not_printed(self):
        with self.assertRaises(AssertionError):
            apex_launchtest.asserts.assertInStdout(
                self.proc_output,
                "default",
                dut_process
            )
