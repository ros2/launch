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


import time
import unittest


class TestResult(unittest.TextTestResult):
    """
    Subclass of unittest.TestResult that collects more information about the tests that ran.

    This class extends TestResult by recording all of the tests that ran, and by recording
    start and stop time for the individual test cases
    """

    def __init__(self, stream=None, descriptions=None, verbosity=None):
        self.__test_cases = {}
        super().__init__(stream, descriptions, verbosity)

    @property
    def testCases(self):
        return self.__test_cases.keys()

    @property
    def testTimes(self):
        """Get a dict of {test_case: elapsed_time}."""
        return {k: v['end'] - v['start'] for (k, v) in self.__test_cases.items()}

    def startTest(self, test):
        self.__test_cases[test] = {
            'start': time.time(),
            'end': 0
        }
        super().startTest(test)

    def stopTest(self, test):
        self.__test_cases[test]['end'] = time.time()
        super().stopTest(test)
