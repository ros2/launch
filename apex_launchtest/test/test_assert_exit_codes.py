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

from apex_launchtest.asserts import assertExitCodes


# To make some duck-typed fake data for assertExitCodes, we just need objects that have
# a 'process_name' string and a 'returncode' integer
class _dummy_proc_data:

    def __init__(self):
        self.process_name = ""
        self.returncode = 0


class TestExitCodes(unittest.TestCase):

    def setUp(self):
        self.dummy_proc_info = []

        for n in range(5):
            proc_data = _dummy_proc_data()
            proc_data.process_name = "process_{}".format(n)
            proc_data.returncode = 0
            self.dummy_proc_info.append(proc_data)

    def test_assert_exit_codes_no_error(self):
        assertExitCodes(self.dummy_proc_info)

    def test_assert_exit_codes_notices_error(self):
        self.dummy_proc_info[2].returncode = 1

        with self.assertRaises(AssertionError) as cm:
            assertExitCodes(self.dummy_proc_info)

        # Check that the process name made it into the error message
        self.assertIn("process_2", str(cm.exception))

    def test_assert_exit_code_allows_specifid_codes(self):
        self.dummy_proc_info[3].returncode = 131

        assertExitCodes(self.dummy_proc_info, allowable_exit_codes=[0, 131])
