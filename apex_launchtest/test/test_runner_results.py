# Copyright 2018 Apex.AI, Inc.
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

import imp
import mock
import os

import ament_index_python
import launch
import launch.actions

from apex_launchtest.apex_runner import ApexRunner


# Run tests on processes that die early with an exit code and make sure the results returned
# indicate failure
def test_dut_with_exception():

    def generate_test_description(ready_fn):
        TEST_PROC_PATH = os.path.join(
            ament_index_python.get_package_prefix('apex_launchtest'),
            'lib/apex_launchtest',
            'terminating_proc'
        )

        return launch.LaunchDescription([
            launch.actions.ExecuteProcess(
                cmd=[TEST_PROC_PATH]
            ),

            launch.actions.OpaqueFunction(function=lambda context: ready_fn()),
        ])

    with mock.patch('apex_launchtest.apex_runner.ApexRunner._run_test'):
        runner = ApexRunner(
            gen_launch_description_fn=generate_test_description,
            test_module=None
        )

        pre_result, post_result = runner.run()

        assert not pre_result.wasSuccessful()
        assert not post_result.wasSuccessful()


# Run some known good tests to check the nominal-good test path
def test_nominally_good_dut():

    # The following is test setup nonsense to turn a string into a python module that can be
    # passed to the apex runner.  You can skip over this.  It does not add to your understanding
    # of the test.
    test_code = """
import unittest
from apex_launchtest import post_shutdown_test

class PreTest(unittest.TestCase):
    def test_pre_ok(self):
        pass

@post_shutdown_test()
class PostTest(unittest.TestCase):
    def test_post_ok(self):
        pass
    """
    module = imp.new_module("test_module")
    exec(test_code, module.__dict__)

    # Here's the actual 'test' part of the test:
    TEST_PROC_PATH = os.path.join(
        ament_index_python.get_package_prefix('apex_launchtest'),
        'lib/apex_launchtest',
        'good_proc'
    )

    def generate_test_description(ready_fn):
        return launch.LaunchDescription([
            launch.actions.ExecuteProcess(
                cmd=[TEST_PROC_PATH]
            ),

            launch.actions.OpaqueFunction(function=lambda context: ready_fn()),
        ])

    runner = ApexRunner(
        gen_launch_description_fn=generate_test_description,
        test_module=module
    )

    pre_result, post_result = runner.run()

    assert pre_result.wasSuccessful()

    assert pre_result.wasSuccessful()
