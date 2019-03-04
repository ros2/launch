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

import apex_launchtest
from apex_launchtest.asserts import assertSequentialStdout


TEST_PROC_PATH = os.path.join(
    ament_index_python.get_package_prefix('apex_launchtest'),
    'lib/apex_launchtest',
    'good_proc'
)


# This launch description shows the prefered way to let the tests access launch actions.  By
# adding them to the test context, it's not necessary to scope them at the module level like in
# the good_proc.test.py example
def generate_test_description(ready_fn):
    # This is necessary to get unbuffered output from the process under test
    proc_env = os.environ.copy()
    proc_env["PYTHONUNBUFFERED"] = "1"

    dut_process = launch.actions.ExecuteProcess(
        cmd=[TEST_PROC_PATH],
        env=proc_env,
    )

    ld = launch.LaunchDescription([
        dut_process,

        # Start tests right away - no need to wait for anything
        launch.actions.OpaqueFunction(function=lambda context: ready_fn()),
    ])

    # Items in this dictionary will be added to the test cases as an attribute based on
    # dictionary key
    test_context = {
        "dut": dut_process,
        "int_val": 10
    }

    return ld, test_context


class TestProcOutput(unittest.TestCase):

    def test_process_output(self):
        # self.dut below was added to the test case because it was part of the test context
        # returned by generate_test_description
        self.proc_output.assertWaitFor("Loop 1", process=self.dut, timeout=10)


@apex_launchtest.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_full_output(self):
        with assertSequentialStdout(self.proc_output, process=self.dut) as cm:
            cm.assertInStdout("Starting Up")
            cm.assertInStdout("Shutting Down")

    def test_int_val(self):
        self.assertEqual(self.int_val, 10)
