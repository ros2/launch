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

from ..util import resolveProcesses


def assertInStdout(proc_output,
                   msg,
                   process,
                   cmd_args=None,
                   *,
                   strict_proc_matching=True):
    """
    Assert that 'msg' was found in the standard out of a process.

    :param proc_output: The process output captured by apex_launchtest.  This is usually injected
    into test cases as self._proc_output
    :type proc_output: An apex_launchtest.IoHandler

    :param msg: The message to search for
    :type msg: string

    :param process: The process whose output will be searched
    :type process: A string (search by process name) or a launch.actions.ExecuteProcess object

    :param cmd_args: Optional.  If 'process' is a string, cmd_args will be used to disambiguate
    processes with the same name.  Pass apex_launchtest.asserts.NO_CMD_ARGS to match a proc without
    command arguments
    :type cmd_args: string

    :param strict_proc_matching: Optional (default True), If proc is a string and the combination
    of proc and cmd_args matches multiple processes, then strict_proc_matching=True will raise
    an error.
    :type strict_proc_matching: bool
    """
    resolved_procs = resolveProcesses(
        info_obj=proc_output,
        process=process,
        cmd_args=cmd_args,
        strict_proc_matching=strict_proc_matching
    )

    for proc in resolved_procs:  # Nominally just one matching proc
        for output in proc_output[proc]:
            if msg in output.text.decode():
                return
    else:
        names = ', '.join(sorted(p.process_details['name'] for p in resolved_procs))
        assert False, "Did not find '{}' in output for any of the matching process {}".format(
            msg,
            names
        )
