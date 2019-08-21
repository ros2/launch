# Copyright 2019 Open Source Robotics Foundation, Inc.
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

import pytest

from ..loader import LoadTestsFromPythonModule
from ..test_runner import LaunchTestRunner


class LaunchTestFailure(Exception):

    def __init__(self, message, results):
        super().__init__()
        self.message = message
        self.results = results

    def __str__(self):
        return self.message


class LaunchTestItem(pytest.Item):

    def __init__(self, name, parent, test_runs, runner_cls=LaunchTestRunner):
        super().__init__(name, parent)
        self.test_runs = test_runs
        self.runner_cls = runner_cls

    def runtest(self):
        launch_args = sum((
            args_set for args_set in self.config.getoption('--launch-args')
        ), [])
        runner = self.runner_cls(
            test_runs=self.test_runs,
            launch_file_arguments=launch_args,
            debug=self.config.getoption('verbose')
        )

        runner.validate()
        results_per_run = runner.run()

        if any(not result.wasSuccessful() for result in results_per_run.values()):
            raise LaunchTestFailure(
                message='some test cases have failed', results=results_per_run
            )

    def repr_failure(self, excinfo):
        if isinstance(excinfo.value, LaunchTestFailure):
            return excinfo.value.message + ':\n' + '\n'.join({
                '{} failed at {}.{}'.format(
                    str(test_run),
                    type(test_case).__name__,
                    test_case._testMethodName
                )
                for test_run, test_result in excinfo.value.results.items()
                for test_case, _ in (test_result.errors + test_result.failures)
                if not test_result.wasSuccessful()
            }) if excinfo.value.results else ''
        return super().repr_failure(excinfo)

    def reportinfo(self):
        return self.fspath, 0, 'launch tests: {}'.format(self.name)


class LaunchTestModule(pytest.File):

    def makeitem(self, *args, **kwargs):
        return LaunchTestItem(*args, **kwargs)

    def collect(self):
        module = self.fspath.pyimport()
        yield self.makeitem(
            name=module.__name__, parent=self,
            test_runs=LoadTestsFromPythonModule(
                module, name=module.__name__
            )
        )


def find_launch_test_entrypoint(path):
    try:
        return getattr(path.pyimport(), 'generate_test_description', None)
    except SyntaxError:
        return None


def pytest_pycollect_makemodule(path, parent):
    entrypoint = find_launch_test_entrypoint(path)
    if entrypoint is not None:
        ihook = parent.session.gethookproxy(path)
        module = ihook.pytest_launch_collect_makemodule(
            path=path, parent=parent, entrypoint=entrypoint
        )
        if module is not None:
            return module
    if path.basename == '__init__.py':
        return pytest.Package(path, parent)
    return pytest.Module(path, parent)


@pytest.hookimpl(trylast=True)
def pytest_launch_collect_makemodule(path, parent, entrypoint):
    marks = getattr(entrypoint, 'pytestmark', [])
    if marks and any(m.name == 'launch_test' for m in marks):
        return LaunchTestModule(path, parent)


def pytest_addhooks(pluginmanager):
    import launch_testing.pytest.hookspecs as hookspecs
    pluginmanager.add_hookspecs(hookspecs)


def pytest_addoption(parser):
    parser.addoption(
        '--launch-args', action='append', nargs='*',
        default=[], help='One or more Launch test arguments'
    )


def pytest_configure(config):
    config.addinivalue_line(
        'markers', 'launch_test: mark a generate_test_description function as a launch test entrypoint'
    )
