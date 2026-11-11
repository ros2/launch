# Copyright 2020 Southwest Research Institute, All Rights Reserved.
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
#
# DISTRIBUTION A. Approved for public release; distribution unlimited.
# OPSEC #4584.

import os

from launch.descriptions.executable import Executable
from launch.launch_context import LaunchContext
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution


def test_executable():
    exe = Executable(cmd='test')
    assert exe is not None


def test_cmd_string_in_list():
    exe = Executable(cmd=['ls "my/subdir/with spaces/"'])
    exe.prepare(LaunchContext(), None)
    assert all(a == b for a, b in zip(exe.final_cmd, ['ls "my/subdir/with spaces/"']))


def test_cmd_strings_in_list():
    exe = Executable(cmd=['ls', '"my/subdir/with spaces/"'])
    exe.prepare(LaunchContext(), None)
    assert all(a == b for a, b in zip(exe.final_cmd, ['ls', '"my/subdir/with spaces/"']))


def test_cmd_multiple_arguments_in_string():
    exe = Executable(cmd=['ls', '-opt1', '-opt2', '-opt3'])
    exe.prepare(LaunchContext(), None)
    assert all(a == b for a, b in zip(exe.final_cmd, ['ls', '-opt1', '-opt2', '-opt3']))


def test_empty_arguments_are_dropped():
    # Regression test for https://github.com/ros2/launch/issues/638
    # Empty arguments (e.g. a substitution expanding to '') must not be
    # passed to the executable as spurious empty argv entries.
    exe = Executable(cmd=['ls', 'foo', '', 'bar'])
    exe.prepare(LaunchContext(), None)
    assert exe.final_cmd == ['ls', 'foo', 'bar']


def test_empty_substitution_argument_is_dropped():
    # The motivating case from #638: a LaunchConfiguration with an empty
    # default value used as a command argument.
    exe = Executable(
        cmd=['prog', 'foo', LaunchConfiguration('bar', default=''), 'baz']
    )
    exe.prepare(LaunchContext(), None)
    assert exe.final_cmd == ['prog', 'foo', 'baz']
    exe = Executable(cmd=['prog', TextSubstitution(text=''), 'x'])
    exe.prepare(LaunchContext(), None)
    assert exe.final_cmd == ['prog', 'x']


def test_empty_executable_is_kept():
    # Only arguments are dropped; the executable slot itself is preserved
    # even if it expands to an empty string.
    exe = Executable(cmd=['', 'arg'])
    exe.prepare(LaunchContext(), None)
    assert exe.final_cmd == ['', 'arg']


def test_empty_arguments_dropped_with_prefix():
    exe = Executable(cmd=['my_program', '', 'arg'], prefix='gdb -ex run --args')
    exe.prepare(LaunchContext(), None)
    assert exe.final_cmd == ['gdb', '-ex', 'run', '--args', 'my_program', 'arg']
    assert exe.final_name.startswith('my_program')


def test_passthrough_properties():
    name = 'name'
    cwd = 'cwd'
    env = {'a': '1'}
    exe = Executable(cmd=['test'], name=name, cwd=cwd, env=env)
    exe.prepare(LaunchContext(), None)
    assert exe.final_name.startswith(name)
    assert exe.final_cwd == cwd
    assert exe.final_env == env


def test_prefix_does_not_overwrite_logger_name():
    # Regression test for https://github.com/ros2/launch/issues/938
    # ``final_name`` (used as the log output prefix) must be derived from the
    # user's command, not from the launch ``prefix=`` token that gets prepended.
    exe = Executable(cmd=['my_program'], prefix='gdb -ex run --args')
    exe.prepare(LaunchContext(), None)
    assert exe.final_cmd[0] == 'gdb'
    assert exe.final_name.startswith('my_program')


def test_substituted_properties():
    os.environ['EXECUTABLE_NAME'] = 'name'
    os.environ['EXECUTABLE_CWD'] = 'cwd'
    os.environ['EXECUTABLE_ENVVAR'] = 'var'
    os.environ['EXECUTABLE_ENVVAL'] = 'value'
    name = EnvironmentVariable('EXECUTABLE_NAME')
    cwd = EnvironmentVariable('EXECUTABLE_CWD')
    env = {EnvironmentVariable('EXECUTABLE_ENVVAR'): EnvironmentVariable('EXECUTABLE_ENVVAL')}
    exe = Executable(cmd=['test'], name=name, cwd=cwd, env=env)
    exe.prepare(LaunchContext(), None)
    assert exe.final_name.startswith('name')
    assert exe.final_cwd == 'cwd'
    assert exe.final_env == {'var': 'value'}
    del os.environ['EXECUTABLE_NAME']
    del os.environ['EXECUTABLE_CWD']
    del os.environ['EXECUTABLE_ENVVAR']
    del os.environ['EXECUTABLE_ENVVAL']
