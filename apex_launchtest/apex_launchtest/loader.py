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


def PreShutdownTestLoader():
    return _make_loader(False)


def PostShutdownTestLoader():
    return _make_loader(True)


def _make_loader(load_post_shutdown):

    class _loader(unittest.TestLoader):
        """TestLoader selectively loads pre-shutdown or post-shutdown tests."""

        def loadTestsFromTestCase(self, testCaseClass):

            if getattr(testCaseClass, "__post_shutdown_test__", False) == load_post_shutdown:
                return super(_loader, self).loadTestsFromTestCase(testCaseClass)
            else:
                # Empty test suites will be ignored by the test runner
                return self.suiteClass()

    return _loader()
