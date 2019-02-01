# Copyright 2019 Apex.AI, Inc.
# All rights reserved.

import inspect
import threading
import unittest

import rclpy
from launch import LaunchService
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.event_handlers import OnProcessIO

from .io_handler import ActiveIoHandler
from .loader import PostShutdownTestLoader, PreShutdownTestLoader
from .proc_info_handler import ActiveProcInfoHandler


class _fail_result(unittest.TestResult):
    """For test runs that fail when the DUT dies unexpectedly."""

    def wasSuccessful(self):
        return False


class ApexRunner(object):

    def __init__(self,
                 gen_launch_description_fn,
                 test_module):
        """
        Create an ApexRunner object.

        :param callable gen_launch_description_fn: A function that returns a ros2 LaunchDesription
        for launching the nodes under test.  This function should take a callable as a parameter
        which will be called when the nodes under test are ready for the test to start
        """
        self._gen_launch_description_fn = gen_launch_description_fn
        self._test_module = test_module
        self._launch_service = LaunchService()
        self._nodes_launched = threading.Event()  # To signal when all nodes started
        self._tests_completed = threading.Event()  # To signal when all the tests have finished

        # Can't run LaunchService.run on another thread :-(
        # See https://github.com/ros2/launch/issues/126
        # Instead, we'll let the tests run on another thread
        self._test_tr = threading.Thread(
            target=self._run_test,
            name="test_runner_thread",
            daemon=True
        )

    def run(self):
        """
        Launch the nodes under test and run the tests.

        :return: A tuple of two unittest.Results - one for tests that ran while nodes were
        active, and another set for tests that ran after nodes were shutdown
        """
        launch_description = self._gen_launch_description_fn(lambda: self._nodes_launched.set())

        # Data to squirrel away for post-shutdown tests
        self.proc_info = ActiveProcInfoHandler()
        self.proc_output = ActiveIoHandler()

        launch_description.add_entity(
            RegisterEventHandler(
                OnProcessExit(on_exit=lambda info, unused: self.proc_info.append(info))
            )
        )

        launch_description.add_entity(
            RegisterEventHandler(
                OnProcessIO(
                    on_stdout=self.proc_output.append,
                    on_stderr=self.proc_output.append,
                )
            )
        )

        self._launch_service.include_launch_description(
            launch_description
        )

        self._test_tr.start()  # Run the tests on another thread
        self._launch_service.run()  # This will block until the test thread stops it

        if not self._tests_completed.wait(timeout=0):
            # LaunchService.run returned before the tests completed.  This can be because the user
            # did ctrl+c, or because all of the launched nodes died before the tests completed

            # We should treat this as a test failure and return some test results indicating such
            print("Nodes under test stopped before tests completed")
            # TODO: This will make the program exit with an exit code, but it's not apparent
            # why.  Consider having apex_rostest_main print some summary
            return _fail_result(), _fail_result()

        # Now, run the post-shutdown tests
        inactive_suite = PostShutdownTestLoader().loadTestsFromModule(self._test_module)
        self._give_attribute_to_tests(self.proc_info, "proc_info", inactive_suite)
        self._give_attribute_to_tests(self.proc_output._io_handler, "proc_output", inactive_suite)
        inactive_results = unittest.TextTestRunner(verbosity=2).run(inactive_suite)

        return self._results, inactive_results

    def validate(self):
        """Inspect the test configuration for configuration errors."""
        # Make sure the function signature of the launch configuration
        # generator is correct
        inspect.getcallargs(self._gen_launch_description_fn, lambda: None)

    def _run_test(self):
        # Waits for the DUT nodes to start (signaled by the _nodes_launched
        # event) and then runs the tests

        if not self._nodes_launched.wait(timeout=15):
            # Timed out waiting for the nodes to start
            print("Timed out waiting for nodes to start up")
            self._launch_service.shutdown()
            return

        node = None
        try:
            # Load the tests
            active_suite = PreShutdownTestLoader().loadTestsFromModule(self._test_module)

            # Start up a ROS2 node which will be made available to the tests
            rclpy.init()
            node = rclpy.create_node("test_node")  # TODO: Give this a better (unique?) name
            self._give_attribute_to_tests(node, "node", active_suite)
            self._give_attribute_to_tests(self.proc_output, "proc_output", active_suite)
            self._give_attribute_to_tests(self.proc_info, "proc_info", active_suite)

            # Run the tests
            self._results = unittest.TextTestRunner(verbosity=2).run(active_suite)

        finally:
            self._tests_completed.set()
            if node:
                node.destroy_node()
            self._launch_service.shutdown()

    def _give_attribute_to_tests(self, data, attr_name, test_suite):
        # Test suites can contain other test suites which will eventually contain
        # the actual test classes to run.  This function will recursively drill down until
        # we find the actual tests and give the tests a reference to the node

        # The effect of this is that every test will have self.node available to it so that
        # it can interact with ROS2 or the process exit coes, or whatever data we want

        try:
            iter(test_suite)
        except TypeError:
            # Base case - test_suite is not iterable, so it must be an individual test method
            setattr(test_suite, attr_name, data)
        else:
            # Otherwise, it's a test_suite, or a list of individual test methods.  recurse
            for test in test_suite:
                self._give_attribute_to_tests(data, attr_name, test)
