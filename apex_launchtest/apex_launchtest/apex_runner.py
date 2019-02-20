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

import inspect
import threading
import unittest

import launch
from launch import LaunchService
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.event_handlers import OnProcessIO
from ros2launch.api.api import parse_launch_arguments

from .io_handler import ActiveIoHandler
from .loader import PostShutdownTestLoader, PreShutdownTestLoader
from .proc_info_handler import ActiveProcInfoHandler
from .test_result import TestResult


class _fail_result(unittest.TestResult):
    """For test runs that fail when the DUT dies unexpectedly."""

    def wasSuccessful(self):
        return False


class ApexRunner(object):

    def __init__(self,
                 gen_launch_description_fn,
                 test_module,
                 launch_file_arguments=[],
                 debug=False):
        """
        Create an ApexRunner object.

        :param callable gen_launch_description_fn: A function that returns a ros2 LaunchDesription
        for launching the processes under test.  This function should take a callable as a
        parameter which will be called when the processes under test are ready for the test to
        start
        """
        self._gen_launch_description_fn = gen_launch_description_fn
        self._test_module = test_module
        self._launch_service = LaunchService(debug=debug)
        self._processes_launched = threading.Event()  # To signal when all processes started
        self._tests_completed = threading.Event()  # To signal when all the tests have finished
        self._launch_file_arguments = launch_file_arguments

        # Can't run LaunchService.run on another thread :-(
        # See https://github.com/ros2/launch/issues/126
        # Instead, we'll let the tests run on another thread
        self._test_tr = threading.Thread(
            target=self._run_test,
            name="test_runner_thread",
            daemon=True
        )

    def get_launch_description(self):
        return self._gen_launch_description_fn(lambda: None)

    def run(self):
        """
        Launch the processes under test and run the tests.

        :return: A tuple of two unittest.Results - one for tests that ran while processes were
        active, and another set for tests that ran after processes were shutdown
        """
        test_ld = self._gen_launch_description_fn(lambda: self._processes_launched.set())

        # Data to squirrel away for post-shutdown tests
        self.proc_info = ActiveProcInfoHandler()
        self.proc_output = ActiveIoHandler()
        parsed_launch_arguments = parse_launch_arguments(self._launch_file_arguments)
        self.test_args = {}
        for k, v in parsed_launch_arguments:
            self.test_args[k] = v

        # Wrap the test_ld in another launch description so we can bind command line arguments to
        # the test and add our own event handlers for process IO and process exit:
        launch_description = LaunchDescription([
            launch.actions.IncludeLaunchDescription(
                launch.LaunchDescriptionSource(launch_description=test_ld),
                launch_arguments=parsed_launch_arguments
            ),
            RegisterEventHandler(
                OnProcessExit(on_exit=lambda info, unused: self.proc_info.append(info))
            ),
            RegisterEventHandler(
                OnProcessIO(
                    on_stdout=self.proc_output.append,
                    on_stderr=self.proc_output.append,
                )
            ),
        ])

        self._launch_service.include_launch_description(
            launch_description
        )

        self._test_tr.start()  # Run the tests on another thread
        self._launch_service.run()  # This will block until the test thread stops it

        if not self._tests_completed.wait(timeout=0):
            # LaunchService.run returned before the tests completed.  This can be because the user
            # did ctrl+c, or because all of the launched nodes died before the tests completed
            print("Processes under test stopped before tests completed")
            self._print_process_output_summary()  # <-- Helpful to debug why processes died early
            # We treat this as a test failure and return some test results indicating such
            return _fail_result(), _fail_result()

        # Now, run the post-shutdown tests
        inactive_suite = PostShutdownTestLoader().loadTestsFromModule(self._test_module)
        self._give_attribute_to_tests(self.proc_info, "proc_info", inactive_suite)
        self._give_attribute_to_tests(self.proc_output._io_handler, "proc_output", inactive_suite)
        self._give_attribute_to_tests(self.test_args, "test_args", inactive_suite)
        inactive_results = unittest.TextTestRunner(
            verbosity=2,
            resultclass=TestResult
        ).run(inactive_suite)

        return self._results, inactive_results

    def validate(self):
        """Inspect the test configuration for configuration errors."""
        # Make sure the function signature of the launch configuration
        # generator is correct
        inspect.getcallargs(self._gen_launch_description_fn, lambda: None)

    def _run_test(self):
        # Waits for the DUT processes to start (signaled by the _processes_launched
        # event) and then runs the tests

        if not self._processes_launched.wait(timeout=15):
            # Timed out waiting for the processes to start
            print("Timed out waiting for processes to start up")
            self._launch_service.shutdown()
            return

        try:
            # Load the tests
            active_suite = PreShutdownTestLoader().loadTestsFromModule(self._test_module)

            self._give_attribute_to_tests(self.proc_output, "proc_output", active_suite)
            self._give_attribute_to_tests(self.proc_info, "proc_info", active_suite)
            self._give_attribute_to_tests(self.test_args, "test_args", active_suite)

            # Run the tests
            self._results = unittest.TextTestRunner(
                verbosity=2,
                resultclass=TestResult
            ).run(active_suite)

        finally:
            self._tests_completed.set()
            self._launch_service.shutdown()

    def _print_process_output_summary(self):
        failed_procs = [proc for proc in self.proc_info if proc.returncode != 0]

        for process in failed_procs:
            print("Process '{}' exited with {}".format(process.process_name, process.returncode))
            print("##### '{}' output #####".format(process.process_name))
            for io in self.proc_output[process.action]:
                print("{}".format(io.text.decode('ascii')))
            print("#" * (len(process.process_name) + 21))

    def _give_attribute_to_tests(self, data, attr_name, test_suite):
        # Test suites can contain other test suites which will eventually contain
        # the actual test classes to run.  This function will recursively drill down until
        # we find the actual tests and give the tests a reference to the process

        # The effect of this is that every test will have `self.attr_name` available to it so that
        # it can interact with ROS2 or the process exit coes, or IO or whatever data we want

        try:
            iter(test_suite)
        except TypeError:
            # Base case - test_suite is not iterable, so it must be an individual test method
            setattr(test_suite, attr_name, data)
        else:
            # Otherwise, it's a test_suite, or a list of individual test methods.  recurse
            for test in test_suite:
                self._give_attribute_to_tests(data, attr_name, test)
