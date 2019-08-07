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
import signal
import sys
import types
import unittest

import ament_index_python
import launch
import launch.actions
import launch.events.process
from launch_testing.loader import LoadTestsFromPythonModule
from launch_testing.test_runner import LaunchTestRunner
import launch_testing.util


def test_wait_for_shutdown():

    def generate_test_description(ready_fn):
        TEST_PROC_PATH = os.path.join(
            ament_index_python.get_package_prefix('launch_testing'),
            'lib/launch_testing',
            'good_proc'
        )

        good_process = launch.actions.ExecuteProcess(
                cmd=[sys.executable, TEST_PROC_PATH],
        )

        # Let 'good_process' run for 10 seconds, then terminate it
        return launch.LaunchDescription([
            good_process,
            launch_testing.util.KeepAliveProc(),
            launch.actions.TimerAction(
                period=10.0,
                actions=[
                    launch.actions.EmitEvent(
                        event=launch.events.process.SignalProcess(
                            signal_number=signal.SIGINT,
                            process_matcher=lambda proc: proc is good_process
                        )
                    )
                ]
            ),
            launch.actions.OpaqueFunction(function=lambda context: ready_fn())
        ]), {'good_process': good_process}

    # This is kind of a weird test-within-a-test, but it's the easiest way to get
    # all of the proc_info handlers hooked up correctly without digging deep into
    # the guts of the runner
    class ProcInfoTests(unittest.TestCase):

        def test_01_check_running_process(self, proc_info, good_process):
            with self.assertRaisesRegexp(AssertionError, 'Timed out waiting for process'):
                proc_info.assertWaitForShutdown(good_process, timeout=3)  # Should raise

        def test_02_check_when_process_exits(self, proc_info, good_process):
            proc_info.assertWaitForShutdown(good_process, timeout=15)

    test_module = types.ModuleType('test_module')
    test_module.generate_test_description = generate_test_description
    test_module.ProcInfoTests = ProcInfoTests

    runner = LaunchTestRunner(
        LoadTestsFromPythonModule(test_module)
    )
    results = runner.run()

    for result in results.values():
        assert result.wasSuccessful()


def test_wait_for_startup():

    def generate_test_description(ready_fn):
        TEST_PROC_PATH = os.path.join(
            ament_index_python.get_package_prefix('launch_testing'),
            'lib/launch_testing',
            'good_proc'
        )

        good_process = launch.actions.ExecuteProcess(
                cmd=[sys.executable, TEST_PROC_PATH],
        )

        # Start 'good_proc' after a ten second timer elapses
        return launch.LaunchDescription([
            launch.actions.TimerAction(
                period=10.0,
                actions=[good_process]
            ),
            launch.actions.OpaqueFunction(function=lambda context: ready_fn())
        ]), {'good_process': good_process}

    # This is kind of a weird test-within-a-test, but it's the easiest way to get
    # all of the proc_info handlers hooked up correctly without digging deep into
    # the guts of the runner
    class ProcInfoTests(unittest.TestCase):

        def test_01_check_for_non_running_process(self, proc_info, good_process):
            with self.assertRaisesRegexp(AssertionError, 'Timed out waiting for process'):
                proc_info.assertWaitForStartup(good_process, timeout=3)  # Should raise

        def test_02_check_when_process_runs(self, proc_info, good_process):
            # The process we're waiting for should start about 7 seconds into this wait
            proc_info.assertWaitForStartup(good_process, timeout=15)

    test_module = types.ModuleType('test_module')
    test_module.generate_test_description = generate_test_description
    test_module.ProcInfoTests = ProcInfoTests

    runner = LaunchTestRunner(
        LoadTestsFromPythonModule(test_module)
    )
    results = runner.run()

    for result in results.values():
        assert result.wasSuccessful()
