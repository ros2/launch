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
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

import ament_index_python

from launch_testing.junitxml import unittestResultsToXml
from launch_testing.test_result import FailResult
from launch_testing.test_result import SkipResult
from launch_testing.test_result import TestResult as TR


class TestGoodXmlOutput(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # For performance, we'll run the test once and generate the XML output, then
        # have multiple test cases assert on it

        cls.tmpdir = tempfile.TemporaryDirectory()
        cls.xml_file = os.path.join(cls.tmpdir.name, 'junit.xml')

        path = os.path.join(
            ament_index_python.get_package_share_directory('launch_testing'),
            'examples',
            'good_proc.test.py'
        )

        assert 0 == subprocess.run(
            args=[
                'launch_test',
                path,
                '--junit-xml', os.path.join(cls.tmpdir.name, 'junit.xml'),
            ],
        ).returncode

    @classmethod
    def tearDownClass(cls):
        cls.tmpdir.cleanup()

    def test_pre_and_post(self):
        tree = ET.parse(self.xml_file)
        root = tree.getroot()

        self.assertEqual(len(root), 1)
        test_suite = root[0]

        # Expecting an element called 'launch' since this was not parametrized
        self.assertEqual(test_suite.attrib['name'], 'launch')

        # Drilling down a little further, we expect the class names to show up in the testcase
        # names
        case_names = [case.attrib['name'] for case in test_suite]
        self.assertIn('test_count_to_four', case_names)
        self.assertIn('test_full_output', case_names)


class TestXmlFunctions(unittest.TestCase):
    # This are closer to unit tests - just call the functions that generate XML

    def unit_test_result_factory(self, test_case_list):
        # Use the unittest library to run some fake test functions and generate a real TestResult
        # that we can serialize to XML

        class TestHost(unittest.TestCase):
            pass

        for n, test_case in enumerate(test_case_list):
            setattr(TestHost, 'test_{}'.format(n), test_case)

        cases = unittest.TestLoader().loadTestsFromTestCase(TestHost)
        with open(os.devnull, 'w') as nullstream:
            runner = unittest.TextTestRunner(
                stream=nullstream,
                resultclass=TR
            )
            return runner.run(cases)

    def test_fail_results_serialize(self):
        xml_tree = unittestResultsToXml(
            name='fail_xml',
            test_results={
                'active_tests': FailResult()
            }
        )

        # Simple sanity check - see that there's a child element called active_tests
        child_names = [chld.attrib['name'] for chld in xml_tree.getroot()]
        self.assertEqual(set(child_names), {'active_tests'})

    def test_skip_results_serialize(self):
        xml_tree = unittestResultsToXml(
            name='skip_xml',
            test_results={
                'active_tests': SkipResult(msg='skip message')
            }
        )

        # Make sure the message got into the 'skip' element
        testsuites_element = xml_tree.getroot()
        testsuite_element = testsuites_element.find('testsuite')
        testcase_element = testsuite_element.find('testcase')
        skip_element = testcase_element.find('skipped')

        self.assertEqual('1', testsuite_element.attrib['skipped'])
        self.assertEqual('skip message', skip_element.attrib['message'])

    def test_multiple_test_results(self):
        xml_tree = unittestResultsToXml(
            name='multiple_launches',
            test_results={
                'launch_1': TR(None, True, 1),
                'launch_2': TR(None, True, 1),
                'launch_3': TR(None, True, 1),
            }
        )

        child_names = [chld.attrib['name'] for chld in xml_tree.getroot()]
        self.assertEqual(set(child_names), {'launch_1', 'launch_2', 'launch_3'})

    def test_result_that_ran(self):
        # This mostly validates the test setup
        dut_xml = unittestResultsToXml(
            test_results={
                'run1': self.unit_test_result_factory([
                    lambda self: None
                ])
            }
        )

        # The expected structure for this test is:
        # <testsuites>
        #   <testsuite name="run1" . . . >
        #     <testcase classname="TestHost" name="test_0" . . . />
        #   </testsuite>
        # <testsuites>

        testsuites_element = dut_xml.getroot()
        testsuite_element = testsuites_element.find('testsuite')
        testcase_element = testsuite_element.find('testcase')

        # The bare minimum XML we require:
        self.assertEqual('0', testsuite_element.attrib['failures'])
        self.assertEqual('0', testsuite_element.attrib['errors'])
        self.assertEqual('1', testsuite_element.attrib['tests'])
        self.assertEqual('test_0', testcase_element.attrib['name'])
        self.assertEqual('TestHost', testcase_element.attrib['classname'])

    def test_result_with_skipped_test(self):

        @unittest.skip('My reason is foo')
        def test_that_is_skipped(self):
            pass  # pragma: no cover

        dut_xml = unittestResultsToXml(
            test_results={
                'run1': self.unit_test_result_factory([
                    test_that_is_skipped
                ])
            }
        )

        # The expected structure for this test is:
        # <testsuites>
        #   <testsuite name="run1" . . . >
        #     <testcase classname="TestHost" name="test_0" . . . >
        #       <skipped message="My reason is foo" . . . />
        #     </testcase>
        #   </testsuite>
        # <testsuites>

        testsuites_element = dut_xml.getroot()
        testsuite_element = testsuites_element.find('testsuite')
        testcase_element = testsuite_element.find('testcase')
        skip_element = testcase_element.find('skipped')

        self.assertEqual('1', testsuite_element.attrib['skipped'])
        self.assertEqual('My reason is foo', skip_element.attrib['message'])

    def test_result_with_failure(self):

        def test_that_fails(self):
            assert 1 == 2

        dut_xml = unittestResultsToXml(
            test_results={
                'run1': self.unit_test_result_factory([
                    test_that_fails
                ])
            }
        )

        # The expected structure for this test is:
        # <testsuites>
        #   <testsuite name="run1" . . . >
        #     <testcase classname="TestHost" name="test_0" . . . >
        #       <failure message="assert 1 == 2" . . .>
        #       </failure>
        #     </testcase>
        #   </testsuite>
        # <testsuites>

        testsuites_element = dut_xml.getroot()
        testsuite_element = testsuites_element.find('testsuite')
        testcase_element = testsuite_element.find('testcase')
        failure_element = testcase_element.find('failure')

        self.assertEqual('1', testsuite_element.attrib['failures'])
        self.assertIn('1 == 2', failure_element.attrib['message'])

    def test_result_with_error(self):

        def test_that_errors(self):
            raise Exception('This is an error')

        dut_xml = unittestResultsToXml(
            test_results={
                'run1': self.unit_test_result_factory([
                    test_that_errors
                ])
            }
        )

        # The expected structure for this test is:
        # <testsuites>
        #   <testsuite name="run1" . . . >
        #     <testcase classname="TestHost" name="test_0" . . . >
        #       <error message="This is an error" . . .>
        #       </failure>
        #     </testcase>
        #   </testsuite>
        # <testsuites>

        testsuites_element = dut_xml.getroot()
        testsuite_element = testsuites_element.find('testsuite')
        testcase_element = testsuite_element.find('testcase')
        error_element = testcase_element.find('error')

        self.assertEqual('1', testsuite_element.attrib['errors'])
        self.assertIn('This is an error', error_element.attrib['message'])

    def test_with_multiple_results(self):

        def good_test(self):
            pass

        def error_test(self):
            raise Exception('I am an error')

        def fail_test(self):
            assert 1 == 2

        dut_xml = unittestResultsToXml(
            test_results={
                'run1': self.unit_test_result_factory([
                    good_test,
                    error_test,
                    fail_test,
                ])
            }
        )

        testsuites_element = dut_xml.getroot()
        testsuite_element = testsuites_element.find('testsuite')

        self.assertEqual('1', testsuite_element.attrib['failures'])
        self.assertEqual('1', testsuite_element.attrib['errors'])
        self.assertEqual('3', testsuite_element.attrib['tests'])
