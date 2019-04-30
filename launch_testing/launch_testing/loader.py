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

import functools
import inspect
import unittest


def _normalize_ld(launch_description_fn):
    # A launch description fn can return just a launch description, or a tuple of
    # (launch_description, test_context).  This wrapper function normalizes things
    # so we always get a tuple, sometimes with an empty dictionary for the test_context
    def wrapper(*args, **kwargs):
        result = launch_description_fn(*args, **kwargs)
        if isinstance(result, tuple):
            return result
        else:
            return result, {}

    return wrapper


class TestRun:

    def __init__(self,
                 test_description_function,
                 param_args,
                 pre_shutdown_tests,
                 post_shutdown_tests):

        self.test_description_function = test_description_function
        self.normalized_test_description = _normalize_ld(test_description_function)

        self.param_args = param_args

        self.pre_shutdown_tests = pre_shutdown_tests
        self.post_shutdown_tests = post_shutdown_tests

    def bind(self, tests, injected_attributes={}, injected_args={}):
        """
        Bind injected_attributes and injected_args to tests.

        Injected Attributes can be accessed from a test as self.name
        Injected Arguments can be accessed as an argument if the test has an argument with a
        matching name
        """
        # Inject test attributes into the test as self.whatever.  This method of giving
        # objects to the test is pretty inferior to injecting them as arguments to the
        # test methods - we may deprecate this in favor of everything being an argument
        for name, value in injected_attributes.items():
            _give_attribute_to_tests(value, name, tests)

        # Give objects with matching names as arguments to tests.  This doesn't have the
        # weird scoping and name collision issues that the above method has.  In fact,
        # we give proc_info and proc_output to the tests as arguments too, so anything
        # you can do with test attributes can also be accomplished with test arguments
        _bind_test_args_to_tests(injected_args, tests)

    def get_launch_description(self):
        """
        Get just the launch description portion of the test_description.

        This should only be used for the purposes of introspecting the launch description.  The
        returned launch description is not meant to be launched
        """
        return self.test_description_function(lambda: None)

    def __str__(self):
        if not self.param_args:
            return 'launch'
        else:
            return 'TODO Parametrize'


def LoadTestsFromPythonModule(module):

    if hasattr(module.generate_test_description, '__parametrized__'):
        normalized_test_description_func = module.generate_test_description
    else:
        normalized_test_description_func = [(module.generate_test_description, {})]

    # If our test description is parameterized, we'll load a set of tests for each
    # individual launch
    return [TestRun(description,
                    args,
                    PreShutdownTestLoader().loadTestsFromModule(module),
                    PostShutdownTestLoader().loadTestsFromModule(module))
            for description, args in normalized_test_description_func]


def PreShutdownTestLoader():
    return _make_loader(False)


def PostShutdownTestLoader():
    return _make_loader(True)


def _make_loader(load_post_shutdown):

    class _loader(unittest.TestLoader):
        """TestLoader selectively loads pre-shutdown or post-shutdown tests."""

        def loadTestsFromTestCase(self, testCaseClass):

            if getattr(testCaseClass, '__post_shutdown_test__', False) == load_post_shutdown:
                cases = super(_loader, self).loadTestsFromTestCase(testCaseClass)
                return cases
            else:
                # Empty test suites will be ignored by the test runner
                return self.suiteClass()

    return _loader()


def _bind_test_args_to_tests(context, test_suite):
    # Look for tests that expect additional arguments and bind items from the context
    # to the tests
    for test in _iterate_tests_in_test_suite(test_suite):
        # Need to reach a little deep into the implementation here to get the test
        # method.  See unittest.TestCase
        test_method = getattr(test, test._testMethodName)
        # Replace the test with a functools.partial that has the arguments
        # provided by the test context already bound
        setattr(
            test,
            test._testMethodName,
            _partially_bind_matching_args(test_method, context)
        )

        test.setUp = _partially_bind_matching_args(
            test.setUp,
            context
        )

        test.tearDown = _partially_bind_matching_args(
            test.tearDown,
            context
        )

    for test_class in _iterate_test_classes_in_test_suite(test_suite):
        test_class.setUpClass = _partially_bind_matching_args(
            test_class.setUpClass,
            context
        )
        test_class.tearDownClass = _partially_bind_matching_args(
            test_class.tearDownClass,
            context
        )


def _partially_bind_matching_args(unbound_function, arg_candidates):
    function_arg_names = inspect.getfullargspec(unbound_function).args
    # We only want to bind the part of the context matches the test args
    matching_args = {k: v for (k, v) in arg_candidates.items() if k in function_arg_names}
    return functools.partial(unbound_function, **matching_args)


def _give_attribute_to_tests(data, attr_name, test_suite):
    # Test suites can contain other test suites which will eventually contain
    # the actual test classes to run.  This function will recursively drill down until
    # we find the actual tests and give the tests a reference to the process

    # The effect of this is that every test will have `self.attr_name` available to it so that
    # it can interact with ROS2 or the process exit coes, or IO or whatever data we want
    for test in _iterate_tests_in_test_suite(test_suite):
        setattr(test, attr_name, data)


def _iterate_test_classes_in_test_suite(test_suite):
    classes = []
    for t in _iterate_tests_in_test_suite(test_suite):
        if t.__class__ not in classes:
            classes.append(t.__class__)
            yield t.__class__


def _iterate_tests_in_test_suite(test_suite):
    try:
        iter(test_suite)
    except TypeError:
        # Base case - test_suite is not iterable, so it must be an individual test method
        yield test_suite
    else:
        # Otherwise, it's a test_suite, or a list of individual test methods.  recurse
        for test in test_suite:
            yield from _iterate_tests_in_test_suite(test)
