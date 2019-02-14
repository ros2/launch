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

# This is necessary to get unbuffered output from the process under test
node_env = os.environ.copy()
node_env["PYTHONUNBUFFERED"] = "1"

dut_process = launch.actions.ExecuteProcess(
    cmd=[TEST_PROC_PATH],
    env=node_env,
)


def generate_test_description(ready_fn):

    return launch.LaunchDescription([
        dut_process,

        # Start tests right away - no need to wait for anything
        launch.actions.OpaqueFunction(function=lambda context: ready_fn()),
    ])


# These tests will run concurrently with the dut process.  After all these tests are done,
# the launch system will shut down the processes that it started up
class TestGoodProcess(unittest.TestCase):

    def test_count_to_four(self):
        # This will match stdout from any process.  In this example there is only one process
        # running
        self.proc_output.assertWaitFor("Loop 1", timeout=10)
        self.proc_output.assertWaitFor("Loop 2", timeout=10)
        self.proc_output.assertWaitFor("Loop 3", timeout=10)
        self.proc_output.assertWaitFor("Loop 4", timeout=10)


@apex_launchtest.post_shutdown_test()
class TestNodeOutput(unittest.TestCase):

    def test_full_output(self):
        # Using the SequentialStdout context manager asserts that the following stdout
        # happened in the same order that it's checked
        with assertSequentialStdout(self.proc_output, dut_process) as cm:
            cm.assertInStdout("Starting Up")
            for n in range(4):
                cm.assertInStdout("Loop {}".format(n))
            cm.assertInStdout("Shutting Down")

    def test_out_of_order(self):
        # This demonstrates that we notice out-of-order IO
        with self.assertRaisesRegexp(AssertionError, "Loop 2 not found"):

            with assertSequentialStdout(self.proc_output, dut_process) as cm:
                cm.assertInStdout("Loop 1")
                cm.assertInStdout("Loop 3")
                cm.assertInStdout("Loop 2")  # This should raise
