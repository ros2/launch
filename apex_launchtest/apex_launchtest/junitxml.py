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


import xml.etree.ElementTree as ET


def unittestResultsToXml(*, name="apex_launchtest", test_results={}):
    """
    Serialize multiple unittest.TestResult objects into an XML document.

    A testSuites element will be the root element of the document.
    """
    # The test_suites element is the top level of the XML result.
    # apex_launchtest results contain two test suites - one from tests that ran while nodes were
    # active, and one from tests that ran after nodes were shut down
    test_suites = ET.Element('testsuites')
    test_suites.set('name', name)
    # test_suites.set('time', ????) skipping 'time' attribute

    # To get tests, failures, and errors, we just want to iterate the results once
    tests = 0
    failures = 0
    errors = 0

    for result in test_results.values():
        tests += result.testsRun
        failures += len(result.failures)
        errors += len(result.errors)

    test_suites.set('tests', str(tests))
    test_suites.set('failures', str(failures))
    test_suites.set('errors', str(errors))

    for (key, value) in test_results.items():
        test_suites.append(unittestResultToXml(key, value))

    return ET.ElementTree(test_suites)


def unittestResultToXml(name, test_result):
    """
    Serialize a single unittest.TestResult to an XML element.

    <testsuite name="str" tests="#" failures="#" errors="#">
      . . .
    </testsuite>
    """
    test_suite = ET.Element('testsuite')
    test_suite.set('name', name)
    test_suite.set('tests', str(test_result.testsRun))
    test_suite.set('failures', str(len(test_result.failures)))
    test_suite.set('errors', str(len(test_result.errors)))
    test_suite.set('skipped', str(len(test_result.skipped)))
    test_suite.set('time', str(round(sum(test_result.testTimes.values()), 3)))

    for case in test_result.testCases:
        test_suite.append(unittestCaseToXml(test_result, case))

    return test_suite


def unittestCaseToXml(test_result, test_case):
    """
    Serialize a unittest.TestCase into an XML element.

    <testcase name="test_1" time="1.05"/>

    Note - an ordinary unittest.TestResult does not record time information.  The TestResult
    class needs to be an apex_launchtest TestResult class
    """
    case_xml = ET.Element('testcase')
    case_xml.set('name', test_case._testMethodName)
    case_xml.set('time', str(round(test_result.testTimes[test_case], 3)))

    for failure in test_result.failures:
        # We're enumerating a list of (test_case, failure_string) tuples here
        if failure[0] == test_case:
            failure_xml = ET.Element('failure')
            failure_xml.set('message', failure[1])
            case_xml.append(failure_xml)

    for error in test_result.errors:
        # Same as above.  (test_case, error_string) tuples
        if error[0] == test_case:
            error_xml = ET.Element('error')
            error_xml.set('message', error[1])
            case_xml.append(error_xml)

    for skip in test_result.skipped:
        if skip[0] == test_case:
            skip_xml = ET.Element('skipped')
            skip_xml.text = skip[1]
            case_xml.append(skip_xml)

    return case_xml
