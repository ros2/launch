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

import types
import unittest

from launch_testing import post_shutdown_test
from launch_testing.loader import LoadTestsFromPythonModule
import pytest


def _source_test_loader(generate_test_description_fn,
                        pre_shutdown_tests=[],
                        post_shutdown_tests=[]):

    # The launch_testing API knows how to load tests from python modules.  Rather than
    # refactoring launch_testing and python unittest, we'll leverage what we already have
    # by stuffing test cases into a module first, then using the existing infrastructure to
    # load it

    test_module = types.ModuleType('test_module')
    test_module.generate_test_description = generate_test_description_fn

    if pre_shutdown_tests:
        class PreShutdownTestClass(unittest.TestCase):
            pass

        for test_func in pre_shutdown_tests:
            setattr(PreShutdownTestClass, test_func.__name__, test_func)

        setattr(test_module, 'PreShutdownTests', PreShutdownTestClass)

    if post_shutdown_tests:
        @post_shutdown_test()
        class PostShutdownTestClass(unittest.TestCase):
            pass

        for test_func in post_shutdown_tests:
            setattr(PostShutdownTestClass, test_func.__name__, test_func)

        setattr(test_module, 'PostShutdownTests', PostShutdownTestClass)

    return LoadTestsFromPythonModule(test_module)


@pytest.fixture
def source_test_loader():

    return _source_test_loader


@pytest.fixture(scope='class')
def source_test_loader_class_fixture(request):

    def wrapper(self, *args, **kwargs):
        return _source_test_loader(*args, **kwargs)

    request.cls.source_test_loader = wrapper
