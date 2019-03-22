# Copyright 2018 Apex.AI, Inc.
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

import imp
import os
import types
import unittest

import ament_index_python
import apex_launchtest
from apex_launchtest.apex_runner import ApexRunner
from apex_launchtest.loader import LoadTestsFromPythonModule
from apex_launchtest.loader import TestRun as TR

import launch
import launch.actions

import mock


# Run tests on processes that die early with an exit code and make sure the results returned
# indicate failure
def test_dut_that_shuts_down(capsys):

    def generate_test_description(ready_fn):
        TEST_PROC_PATH = os.path.join(
            ament_index_python.get_package_prefix('apex_launchtest'),
            'lib/apex_launchtest',
            'terminating_proc'
        )

        return launch.LaunchDescription([
            launch.actions.ExecuteProcess(
                cmd=[TEST_PROC_PATH]
            ),

            launch.actions.OpaqueFunction(function=lambda context: ready_fn()),
        ])

    with mock.patch('apex_launchtest.apex_runner._RunnerWorker._run_test'):
        runner = ApexRunner(
            [TR(generate_test_description, {}, [], [])]
        )

        results = runner.run()

        for result in results.values():
            assert not result.wasSuccessful()

    # This is the negative version of the test below.  If no exit code, no extra output
    # is generated
    out, err = capsys.readouterr()
    assert 'Starting Up' not in out


def test_dut_that_has_exception(capsys):
    # This is the same as above, but we also want to check we get extra output from processes
    # that had an exit code

    def generate_test_description(ready_fn):
        TEST_PROC_PATH = os.path.join(
            ament_index_python.get_package_prefix('apex_launchtest'),
            'lib/apex_launchtest',
            'terminating_proc'
        )

        EXIT_PROC_PATH = os.path.join(
            ament_index_python.get_package_prefix('apex_launchtest'),
            'lib/apex_launchtest',
            'exit_code_proc'
        )

        return launch.LaunchDescription([
            launch.actions.ExecuteProcess(
                cmd=[TEST_PROC_PATH, '--exception']
            ),

            # This process makes sure we can handle processes that exit with a code but don't
            # generate any output.  This is a regression test for an issue fixed in PR31
            launch.actions.ExecuteProcess(
                cmd=[EXIT_PROC_PATH, '--silent']
            ),

            launch.actions.OpaqueFunction(function=lambda context: ready_fn()),
        ])

    with mock.patch('apex_launchtest.apex_runner._RunnerWorker._run_test'):
        runner = ApexRunner(
            [TR(generate_test_description, {}, [], [])]
        )

        results = runner.run()

        for result in results.values():
            assert not result.wasSuccessful()

    # Make sure some information about WHY the process died shows up in the output
    out, err = capsys.readouterr()
    assert 'Starting Up' in out
    assert 'Process had a pretend error' in out  # This is the exception text from exception_node


# Run some known good tests to check the nominal-good test path
def test_nominally_good_dut():

    # The following is test setup nonsense to turn a string into a python module that can be
    # passed to the apex runner.  You can skip over this.  It does not add to your understanding
    # of the test.
    test_code = """
import unittest
from apex_launchtest import post_shutdown_test

class PreTest(unittest.TestCase):
    def test_pre_ok(self):
        pass

@post_shutdown_test()
class PostTest(unittest.TestCase):
    def test_post_ok(self):
        pass
    """
    module = imp.new_module('test_module')
    exec(test_code, module.__dict__)

    # Here's the actual 'test' part of the test:
    TEST_PROC_PATH = os.path.join(
        ament_index_python.get_package_prefix('apex_launchtest'),
        'lib/apex_launchtest',
        'good_proc'
    )

    def generate_test_description(ready_fn):
        return launch.LaunchDescription([
            launch.actions.ExecuteProcess(
                cmd=[TEST_PROC_PATH]
            ),

            launch.actions.OpaqueFunction(function=lambda context: ready_fn()),
        ])

    module.generate_test_description = generate_test_description

    runner = ApexRunner(
        LoadTestsFromPythonModule(module)
    )

    results = runner.run()

    for result in results.values():
        assert result.wasSuccessful()


def test_parametrized_run_with_one_failure():

    # Test Data
    @apex_launchtest.parametrize('arg_val', [1, 2, 3, 4, 5])
    def generate_test_description(arg_val, ready_fn):
        TEST_PROC_PATH = os.path.join(
            ament_index_python.get_package_prefix('apex_launchtest'),
            'lib/apex_launchtest',
            'good_proc'
        )

        # This is necessary to get unbuffered output from the process under test
        proc_env = os.environ.copy()
        proc_env['PYTHONUNBUFFERED'] = '1'

        return launch.LaunchDescription([
            launch.actions.ExecuteProcess(
                cmd=[TEST_PROC_PATH],
                env=proc_env,
            ),
            launch.actions.OpaqueFunction(function=lambda context: ready_fn())
        ])

    class FakePreShutdownTests(unittest.TestCase):

        def test_fail_on_two(self, proc_output, arg_val):
            proc_output.assertWaitFor('Starting Up')
            assert arg_val != 2

    @apex_launchtest.post_shutdown_test()
    class FakePostShutdownTests(unittest.TestCase):

        def test_fail_on_three(self, arg_val):
            assert arg_val != 3

    # Set up a fake module containing the test data:
    test_module = types.ModuleType('test_module')
    test_module.generate_test_description = generate_test_description
    test_module.FakePreShutdownTests = FakePreShutdownTests
    test_module.FakePostShutdownTests = FakePostShutdownTests

    # Run the test:
    runner = ApexRunner(
        LoadTestsFromPythonModule(test_module)
    )

    results = runner.run()

    passes = [result for result in results.values() if result.wasSuccessful()]
    fails = [result for result in results.values() if not result.wasSuccessful()]

    assert len(passes) == 3  # 1, 4, and 5 should pass
    assert len(fails) == 2  # 2 fails in an active test, 3 fails in a post-shutdown test
