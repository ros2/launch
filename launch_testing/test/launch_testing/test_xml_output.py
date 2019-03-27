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

        self.assertEqual(len(root.getchildren()), 1)
        test_suite = root.getchildren()[0]

        # Expecting an element called 'launch' since this was not parametrized
        self.assertEqual(test_suite.attrib['name'], 'launch')

        # Drilling down a little further, we expect the class names to show up in the testcase
        # names
        case_names = [case.attrib['name'] for case in test_suite.getchildren()]
        self.assertIn('TestGoodProcess.test_count_to_four', case_names)
        self.assertIn('TestProcessOutput.test_full_output', case_names)


class TestXmlFunctions(unittest.TestCase):
    # This are closer to unit tests - just call the functions that generate XML

    def test_fail_results_serialize(self):
        xml_tree = unittestResultsToXml(
            name='fail_xml',
            test_results={
                'active_tests': FailResult()
            }
        )

        # Simple sanity check - see that there's a child element called active_tests
        child_names = [chld.attrib['name'] for chld in xml_tree.getroot().getchildren()]
        self.assertEqual(set(child_names), {'active_tests'})

    def test_multiple_test_results(self):
        xml_tree = unittestResultsToXml(
            name='multiple_launches',
            test_results={
                'launch_1': TR(None, True, 1),
                'launch_2': TR(None, True, 1),
                'launch_3': TR(None, True, 1),
            }
        )

        child_names = [chld.attrib['name'] for chld in xml_tree.getroot().getchildren()]
        self.assertEqual(set(child_names), {'launch_1', 'launch_2', 'launch_3'})
