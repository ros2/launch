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

from launch.descriptions.executable import Executable
from launch.launch_context import LaunchContext


def test_executable():
    exe = Executable(cmd="test")
    assert exe is not None


def test_cmd_string_in_list():
    exe = Executable(cmd=['ls "my/subdir/with spaces/"'])
    exe.apply_context(LaunchContext())
    assert all([a == b for a, b in zip(exe.final_cmd, ['ls "my/subdir/with spaces/"'])])


def test_cmd_strings_in_list():
    exe = Executable(cmd=['ls', '"my/subdir/with spaces/"'])
    exe.apply_context(LaunchContext())
    assert all([a == b for a, b in zip(exe.final_cmd, ['ls', '"my/subdir/with spaces/"'])])


def test_cmd_multiple_arguments_in_string():
    exe = Executable(cmd=['ls', '-opt1', '-opt2', '-opt3'])
    exe.apply_context(LaunchContext())
    assert all([a == b for a, b in zip(exe.final_cmd, ['ls', '-opt1', '-opt2', '-opt3'])])
