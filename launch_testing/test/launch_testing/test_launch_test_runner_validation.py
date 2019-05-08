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

import imp
import unittest

import launch_testing
from launch_testing.loader import LoadTestsFromPythonModule
from launch_testing.test_runner import LaunchTestRunner


def make_test_run_for_dut(generate_test_description_function):
    module = imp.new_module('test_module')
    module.generate_test_description = generate_test_description_function
    return LoadTestsFromPythonModule(module)


class TestLaunchTestRunnerValidation(unittest.TestCase):

    def test_catches_bad_signature(self):

        dut = LaunchTestRunner(
            make_test_run_for_dut(
                lambda: None
            )
        )

        with self.assertRaises(TypeError):
            dut.validate()

        dut = LaunchTestRunner(
            make_test_run_for_dut(
                lambda ready_fn: None
            )
        )

        dut.validate()

    def test_too_many_arguments(self):

        dut = LaunchTestRunner(
            make_test_run_for_dut(lambda ready_fn, extra_arg: None)
        )

        with self.assertRaisesRegex(Exception, "unexpected extra argument 'extra_arg'"):
            dut.validate()

    def test_bad_parametrization_argument(self):

        @launch_testing.parametrize('bad_argument', [1, 2, 3])
        def bad_launch_description(ready_fn):
            pass  # pragma: no cover

        dut = LaunchTestRunner(
            make_test_run_for_dut(bad_launch_description)
        )

        with self.assertRaisesRegex(Exception, 'Could not find an argument') as cm:
            dut.validate()
        self.assertIn('bad_argument', str(cm.exception))
