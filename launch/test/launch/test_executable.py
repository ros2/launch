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
#
# Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS
# Part 252.227-7013 or 7014 (Feb 2014).
#
# This notice must appear in all copies of this file and its derivatives.

import os

from launch.descriptions.executable import Executable
from launch.launch_context import LaunchContext
from launch.substitutions import EnvironmentVariable


def test_executable():
    exe = Executable(cmd='test')
    assert exe is not None


def test_cmd_string_in_list():
    exe = Executable(cmd=['ls "my/subdir/with spaces/"'])
    exe.prepare(None, LaunchContext())
    assert all(a == b for a, b in zip(exe.final_cmd, ['ls "my/subdir/with spaces/"']))


def test_cmd_strings_in_list():
    exe = Executable(cmd=['ls', '"my/subdir/with spaces/"'])
    exe.prepare(None, LaunchContext())
    assert all(a == b for a, b in zip(exe.final_cmd, ['ls', '"my/subdir/with spaces/"']))


def test_cmd_multiple_arguments_in_string():
    exe = Executable(cmd=['ls', '-opt1', '-opt2', '-opt3'])
    exe.prepare(None, LaunchContext())
    assert all(a == b for a, b in zip(exe.final_cmd, ['ls', '-opt1', '-opt2', '-opt3']))


def test_passthrough_properties():
    name = 'name'
    cwd = 'cwd'
    env = {'a': '1'}
    exe = Executable(cmd=['test'], name=name, cwd=cwd, env=env)
    exe.prepare(None, LaunchContext())
    assert exe.final_name.startswith(name)
    assert exe.final_cwd == cwd
    assert exe.final_env == env


def test_substituted_properties():
    os.environ['EXECUTABLE_NAME'] = 'name'
    os.environ['EXECUTABLE_CWD'] = 'cwd'
    os.environ['EXECUTABLE_ENVVAR'] = 'var'
    os.environ['EXECUTABLE_ENVVAL'] = 'value'
    name = EnvironmentVariable('EXECUTABLE_NAME')
    cwd = EnvironmentVariable('EXECUTABLE_CWD')
    env = {EnvironmentVariable('EXECUTABLE_EV'): EnvironmentVariable('EXECUTABLE_ENVVAL')}
    exe = Executable(cmd=['test'], name=name, cwd=cwd, env=env)
    exe.prepare(None, LaunchContext())
    assert exe.final_name.startswith('name')
    assert exe.final_cwd == 'cwd'
    assert exe.final_env == {'var': 'value'}
    del os.environ['EXECUTABLE_NAME']
    del os.environ['EXECUTABLE_CWD']
    del os.environ['EXECUTABLE_ENVVAR']
    del os.environ['EXECUTABLE_ENVVAL']
