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

import apex_launchtest
import apex_launchtest.util
import launch.actions
import launch.substitutions


class TestResolveProcess(unittest.TestCase):

    def test_unlaunched_process_lookup(self):
        # Regression test for https://github.com/ApexAI/apex_rostest/issues/35

        info_obj = apex_launchtest.ProcInfoHandler()

        lookup_obj = launch.actions.ExecuteProcess(
            cmd=[
                'python',
                '-c',
                '',
            ]
        )

        with self.assertRaises(apex_launchtest.util.NoMatchingProcessException) as cm:
            apex_launchtest.util.resolveProcesses(
                info_obj,
                process=lookup_obj
            )

        # We'll get a good error mesasge here because there were no substitutions in
        # the execute process cmd - it's all text
        self.assertIn('python -c', str(cm.exception))

    def test_unlaunched_process_lookup_with_substitutions(self):
        info_obj = apex_launchtest.ProcInfoHandler()

        lookup_obj = launch.actions.ExecuteProcess(
            cmd=[
                launch.substitutions.LocalSubstitution('foo'),
                'python',
                '-c',
                '',
            ]
        )

        with self.assertRaises(apex_launchtest.util.NoMatchingProcessException) as cm:
            apex_launchtest.util.resolveProcesses(
                info_obj,
                process=lookup_obj
            )

        # Since the process wasn't launched yet, and it has substitutions that need to be
        # resolved by the launch system, we won't be able to take a guess at the command
        self.assertIn('Unknown', str(cm.exception))
