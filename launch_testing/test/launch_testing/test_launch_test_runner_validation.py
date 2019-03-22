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

import unittest

from launch_testing.loader import TestRun as TR  # Named TR so pytest doesn't think it's a test
from launch_testing.test_runner import LaunchTestRunner


class TestLaunchTestRunnerValidation(unittest.TestCase):

    def test_catches_bad_signature(self):

        dut = LaunchTestRunner(
            [TR(
                test_description_function=lambda: None,
                param_args={},
                pre_shutdown_tests=None,
                post_shutdown_tests=None,
            )]
        )

        with self.assertRaises(TypeError):
            dut.validate()

        dut = LaunchTestRunner(
            [TR(
                test_description_function=lambda ready_fn: None,
                param_args={},
                pre_shutdown_tests=None,
                post_shutdown_tests=None,
            )]
        )

        dut.validate()
