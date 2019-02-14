# Copyright 2019 Apex.AI, Inc.
# All rights reserved.

import time
import unittest


class TestResult(unittest.TestResult):
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
