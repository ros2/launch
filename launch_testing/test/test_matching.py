# Copyright 2015 Open Source Robotics Foundation, Inc.
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
import sys
import tempfile

from nose.tools import assert_raises

from launch import LaunchDescriptor
from launch.exit_handler import ignore_exit_handler
from launch.launcher import DefaultLauncher
import launch_testing
from launch_testing import create_handler, UnmatchedOutputError


def _run_launch_testing(
        output_file, prepended_lines=False, appended_lines=False, interleaved_lines=False,
        filtered_prefixes=None):
    output_handlers = []

    launch_descriptor = LaunchDescriptor()

    name = "test_executable_0"

    handler = create_handler(
        name, launch_descriptor, output_file,
        filtered_prefixes=filtered_prefixes)

    assert handler, 'cannot find appropriate handler for %s' % output_file

    output_handlers.append(handler)

    executable_command = [
        sys.executable,
        os.path.join(os.path.abspath(os.path.dirname(__file__)), 'matching.py')]

    if prepended_lines:
        executable_command.append('--prepended-lines')

    if appended_lines:
        executable_command.append('--appended-lines')

    if interleaved_lines:
        executable_command.append('--interleaved-lines')

    launch_descriptor.add_process(
        cmd=executable_command,
        name=name,
        exit_handler=ignore_exit_handler,
        output_handlers=output_handlers)

    launcher = DefaultLauncher()
    launcher.add_launch_descriptor(launch_descriptor)
    rc = launcher.launch()

    assert rc == 0, \
        "the launch file failed with exit code '{0}'".format(rc)

    for handler in output_handlers:
        try:
            handler.check()
        except UnmatchedOutputError:
            raise


def test_matching_text():
    # this temporary directory and files contained in it will be deleted when the process ends.
    tempdir = tempfile.mkdtemp()
    output_file = os.path.join(tempdir, "testfile")
    full_output_file = output_file + ".txt"
    with open(full_output_file, 'w+') as f:
        f.write('this is line 1\nthis is line b')

    # regex is matched exactly
    _run_launch_testing(output_file)

    # unmatched lines appear before expected text
    with assert_raises(UnmatchedOutputError):
        _run_launch_testing(output_file, prepended_lines=True)

    # unmatched lines appear after expected text
    with assert_raises(UnmatchedOutputError):
        _run_launch_testing(output_file, appended_lines=True)

    # filtered lines appear before expected text
    filtered_prefixes = launch_testing.get_default_filtered_prefixes()
    filtered_prefixes.append(b'license')
    _run_launch_testing(output_file, prepended_lines=True, filtered_prefixes=filtered_prefixes)

    # unmatched lines appear interleaved with expected text
    with assert_raises(UnmatchedOutputError):
        _run_launch_testing(output_file, interleaved_lines=True)

    # filtered lines appear interleaved with expected text
    filtered_prefixes = launch_testing.get_default_filtered_prefixes()
    filtered_prefixes.append(b'debug')
    _run_launch_testing(output_file, interleaved_lines=True, filtered_prefixes=filtered_prefixes)


def test_matching_regex():
    # this temporary directory and files contained in it will be deleted when the process ends.
    tempdir = tempfile.mkdtemp()
    output_file = os.path.join(tempdir, "testfile")
    full_output_file = output_file + ".regex"
    with open(full_output_file, 'w+') as f:
        f.write('this is line \d\nthis is line [a-z]')

    # regex is matched exactly
    _run_launch_testing(output_file)

    # unmatched lines appear before regex is matched
    with assert_raises(UnmatchedOutputError):
        _run_launch_testing(output_file, prepended_lines=True)

    # unmatched lines appear after regex is matched
    with assert_raises(UnmatchedOutputError):
        _run_launch_testing(output_file, appended_lines=True)

    # filtered lines appear before regex is matched
    filtered_prefixes = launch_testing.get_default_filtered_prefixes()
    filtered_prefixes.append(b'license')
    _run_launch_testing(output_file, prepended_lines=True, filtered_prefixes=filtered_prefixes)

    # unmatched lines appear interleaved with regex lines 
    with assert_raises(UnmatchedOutputError):
        _run_launch_testing(output_file, interleaved_lines=True)

    # filtered lines appear interleaved with regex lines
    filtered_prefixes = launch_testing.get_default_filtered_prefixes()
    filtered_prefixes.append(b'debug')
    _run_launch_testing(output_file, interleaved_lines=True, filtered_prefixes=filtered_prefixes)


if __name__ == '__main__':
    test_matching_regex()
    test_matching_text()
