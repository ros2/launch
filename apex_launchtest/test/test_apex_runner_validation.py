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

from apex_launchtest import ApexRunner


class TestApexRunnerValidation(unittest.TestCase):

    def test_catches_bad_signature(self):

        dut = ApexRunner(
            gen_launch_description_fn=lambda: None,
            test_module=None
        )

        with self.assertRaises(TypeError):
            dut.validate()

        dut = ApexRunner(
            gen_launch_description_fn=lambda fn: None,
            test_module=None
        )

        dut.validate()
