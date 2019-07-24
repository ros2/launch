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

from osrf_pycommon.terminal_color import remove_ansi_escape_senquences

from ..util import resolveProcesses


def normalize_lineseps(lines):
    r"""Normalize and then return the given lines to all use '\n'."""
    lines = lines.replace(os.linesep, '\n')
    # This happens (even on Linux and macOS) when capturing I/O from an
    # emulated tty.
    lines = lines.replace('\r\n', '\n')
    return lines


def get_matching_function(expected_output):
    if isinstance(expected_output, (list, tuple)):
        if len(expected_output) > 0:
            if isinstance(expected_output[0], str):
                def _match(expected, actual):
                    lines = actual.splitlines()
                    for pattern in expected:
                        if pattern not in lines:
                            return False
                        index = lines.index(pattern)
                        lines = lines[index + 1:]
                    return True
                return _match
            if hasattr(expected_output[0], 'search'):
                def _match(expected, actual):
                    start = 0
                    actual = normalize_lineseps(actual)
                    for pattern in expected:
                        match = pattern.search(actual, start)
                        if match is None:
                            return False
                        start = match.end()
                    return True
                return _match
    elif isinstance(expected_output, str):
        return lambda expected, actual: expected in actual
    elif hasattr(expected_output, 'search'):
        return lambda expected, actual: (
            expected.search(normalize_lineseps(actual)) is not None
        )
    raise ValueError('Unknown format for expected output')


def assertInStdout(proc_output,
                   expected_output,
                   process,
                   cmd_args=None,
                   *,
                   output_filter=None,
                   strict_proc_matching=True,
                   strip_ansi_escape_sequences=True):
    """
    Assert that 'output' was found in the standard out of a process.

    :param proc_output: The process output captured by launch_test.  This is usually injected
    into test cases as self._proc_output
    :type proc_output: An launch_testing.IoHandler

    :param expected_output: The output to search for
    :type expected_output: string or regex pattern or list of the aforementioned types

    :param process: The process whose output will be searched
    :type process: A string (search by process name) or a launch.actions.ExecuteProcess object

    :param cmd_args: Optional.  If 'process' is a string, cmd_args will be used to disambiguate
    processes with the same name.  Pass launch_testing.asserts.NO_CMD_ARGS to match a proc without
    command arguments
    :type cmd_args: string

    :param output_filter: Optional. A function to filter output before attempting any assertion.
    :type output_filter: callable

    :param strict_proc_matching: Optional (default True), If proc is a string and the combination
    of proc and cmd_args matches multiple processes, then strict_proc_matching=True will raise
    an error.
    :type strict_proc_matching: bool

    :param strip_ansi_escape_sequences: If True (default), strip ansi escape
    sequences from actual output before comparing with the output filter or
    expected output.
    :type strip_ansi_escape_sequences: bool
    """
    resolved_procs = resolveProcesses(
        info_obj=proc_output,
        process=process,
        cmd_args=cmd_args,
        strict_proc_matching=strict_proc_matching
    )
    if output_filter is not None:
        if not callable(output_filter):
            raise ValueError('output_filter is not callable')
    output_match = get_matching_function(expected_output)

    for proc in resolved_procs:  # Nominally just one matching proc
        full_output = ''.join(
            output.text.decode() for output in proc_output[proc] if output.from_stdout
        )
        if strip_ansi_escape_sequences:
            full_output = remove_ansi_escape_senquences(full_output)
        if output_filter is not None:
            full_output = output_filter(full_output)
        if output_match(expected_output, full_output):
            break
    else:
        names = ', '.join(sorted(p.process_details['name'] for p in resolved_procs))
        assert False, "Did not find '{}' in output for any of the matching processes: {}".format(
            expected_output, names
        )
