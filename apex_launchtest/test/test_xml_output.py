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


class TestGoodXmlOutput(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # For performance, we'll run the test once and generate the XML output, then
        # have multiple test cases assert on it

        cls.tmpdir = tempfile.TemporaryDirectory()
        cls.xml_file = os.path.join(cls.tmpdir.name, 'junit.xml')

        path = os.path.join(
            ament_index_python.get_package_share_directory('apex_launchtest'),
            'examples',
            'good_proc.test.py'
        )

        assert 0 == subprocess.run(
            args=[
                'apex_launchtest',
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

        self.assertEqual(len(root.getchildren()), 2)

        # Expecting an element called "active_tests" and "after_shutdown_tests"
        child_names = [chld.attrib['name'] for chld in root.getchildren()]
        self.assertEqual(set(child_names), set(['active_tests', 'after_shutdown_tests']))
