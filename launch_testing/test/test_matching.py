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
from __future__ import print_function

import os
import sys
import tempfile

from nose.tools import assert_raises

from launch import LaunchDescriptor
from launch.exit_handler import ignore_exit_handler
from launch.launcher import DefaultLauncher
from launch.output_handler import ConsoleOutput
import launch_testing
from launch_testing import create_handler, UnmatchedOutputError


def _run_launch_testing(
        output_file, prepended_lines=False, appended_lines=False, interleaved_lines=False,
        filtered_prefixes=None, no_output=False, exact_match=True, exit_on_match=False,
        reprints=0):
    output_handlers = [ConsoleOutput()]

    launch_descriptor = LaunchDescriptor()

    name = 'test_executable_0'

    handler = create_handler(
        name, launch_descriptor, output_file, exit_on_match=exit_on_match,
        filtered_prefixes=filtered_prefixes, exact_match=exact_match)

    assert handler, 'cannot find appropriate handler for %s' % output_file

    output_handlers.append(handler)

    executable_command = [
        sys.executable, '-u',
        os.path.join(os.path.abspath(os.path.dirname(__file__)), 'matching.py')]

    if no_output:
        executable_command.append('--no-output')

    if prepended_lines:
        executable_command.append('--prepended-lines')

    if appended_lines:
        executable_command.append('--appended-lines')

    if interleaved_lines:
        executable_command.append('--interleaved-lines')

    executable_command.append('--reprints')
    executable_command.append(str(reprints))

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

    try:
        handler.check()
    except UnmatchedOutputError:
        raise


def test_matching_text():
    # this temporary directory and files contained in it will be deleted when the process ends.
    tempdir = tempfile.mkdtemp()
    output_file = os.path.join(tempdir, 'testfile')
    full_output_file = output_file + '.txt'
    with open(full_output_file, 'w+') as f:
        f.write('this is line 1\nthis is line b\n')

    print('Testing when expected text is never printed.')
    with assert_raises(UnmatchedOutputError):
        _run_launch_testing(output_file, no_output=True)

    print('Testing when expected text appears exactly.')
    _run_launch_testing(output_file)

    print('Testing when unmatched lines appear before expected text.')
    with assert_raises(UnmatchedOutputError):
        _run_launch_testing(output_file, prepended_lines=True)

    print('Testing when unmatched lines appear after expected text.')
    with assert_raises(UnmatchedOutputError):
        _run_launch_testing(output_file, appended_lines=True)

    print('Testing when filtered lines appear before expected text.')
    filtered_prefixes = launch_testing.get_default_filtered_prefixes()
    filtered_prefixes.append(b'license')
    _run_launch_testing(output_file, prepended_lines=True, filtered_prefixes=filtered_prefixes)

    print('Testing when unmatched lines appear interleaved with expected text.')
    with assert_raises(UnmatchedOutputError):
        _run_launch_testing(output_file, interleaved_lines=True)

    print('Testing when filtered lines appear interleaved with expected text.')
    filtered_prefixes = launch_testing.get_default_filtered_prefixes()
    filtered_prefixes.append(b'debug')
    _run_launch_testing(output_file, interleaved_lines=True, filtered_prefixes=filtered_prefixes)

    print(
        'Testing when unmatched lines appear before/after text, '
        'but exact matching is not required.')
    _run_launch_testing(output_file, prepended_lines=True, appended_lines=True, exact_match=False)

    print(
        'Testing when unmatched lines appear interleaved with text lines, '
        'but exact matching is not required.')
    with assert_raises(UnmatchedOutputError):
        _run_launch_testing(output_file, interleaved_lines=True, exact_match=False)

    print('Testing exiting upon text match.')
    _run_launch_testing(output_file, reprints=-1, exit_on_match=True)


def test_matching_regex():
    # this temporary directory and files contained in it will be deleted when the process ends.
    tempdir = tempfile.mkdtemp()
    output_file = os.path.join(tempdir, 'testfile')
    full_output_file = output_file + '.regex'
    with open(full_output_file, 'w+') as f:
        f.write('this is line \d\nthis is line [a-z]\n')

    print('Testing when regex is never matched.')
    with assert_raises(UnmatchedOutputError):
        _run_launch_testing(output_file, no_output=True)

    print('Testing when regex match appears exactly.')
    _run_launch_testing(output_file)

    print('Testing when unmatched lines appear before regex is matched.')
    with assert_raises(UnmatchedOutputError):
        _run_launch_testing(output_file, prepended_lines=True)

    print('Testing when unmatched lines appear after regex is matched.')
    with assert_raises(UnmatchedOutputError):
        _run_launch_testing(output_file, appended_lines=True)

    print('Testing when filtered lines appear before regex is matched.')
    filtered_prefixes = launch_testing.get_default_filtered_prefixes()
    filtered_prefixes.append(b'license')
    _run_launch_testing(output_file, prepended_lines=True, filtered_prefixes=filtered_prefixes)

    print('Testing when unmatched lines appear interleaved with regex lines.')
    with assert_raises(UnmatchedOutputError):
        _run_launch_testing(output_file, interleaved_lines=True)

    print('Testing when filtered lines appear interleaved with regex lines.')
    filtered_prefixes = launch_testing.get_default_filtered_prefixes()
    filtered_prefixes.append(b'debug')
    _run_launch_testing(output_file, interleaved_lines=True, filtered_prefixes=filtered_prefixes)

    print(
        'Testing when unmatched lines appear before/after regex lines, '
        'but exact matching is not required.')
    _run_launch_testing(output_file, prepended_lines=True, appended_lines=True, exact_match=False)

    print(
        'Testing when unmatched lines appear interleaved with regex lines, '
        'but exact matching is not required.')
    with assert_raises(UnmatchedOutputError):
        _run_launch_testing(output_file, interleaved_lines=True, exact_match=False)

    print('Testing exiting upon regex match.')
    _run_launch_testing(output_file, reprints=-1, exit_on_match=True)

    # Test regex which can match multiple times
    output_file = os.path.join(tempdir, 'testfile2')
    full_output_file = output_file + '.regex'
    with open(full_output_file, 'w+') as f:
        f.write('(this is line \d\nthis is line [a-z]\n)+')

    print('Testing when multiple regex matches are made.')
    _run_launch_testing(output_file)
    _run_launch_testing(output_file, reprints=1)

    # Test regex which has matches for defined groups
    output_file = os.path.join(tempdir, 'testfile2')
    full_output_file = output_file + '.regex'
    with open(full_output_file, 'w+') as f:
        f.write('this is (\w+) \d\nthis is \\1 [a-z]\n')

    print('Testing when the regex has groups.')
    _run_launch_testing(output_file)


if __name__ == '__main__':
    test_matching_regex()
    test_matching_text()
