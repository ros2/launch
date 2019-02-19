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

import launch.actions

NO_CMD_ARGS = object()


def _proc_to_name_and_args(proc):
    # proc is a launch.actions.ExecuteProcess
    return "{} {}".format(
        proc.process_details['name'],
        " ".join(proc.process_details['cmd'][1:])
    )


def _assertInStdoutByProcessAction(
        proc_output,
        msg,
        process_action):

    for output in proc_output[process_action]:
        if msg in output.text.decode('ascii'):
            return
    else:
        assert False, "Did not find '{}' in output for {}".format(
            msg,
            _proc_to_name_and_args(process_action)
        )


def _assertInStdoutByStringProcessName(
        proc_output,
        msg,
        proc_name,
        cmd_args,
        strict_proc_matching):

    # Ensure that the combination proc_name and cmd_args are not ambiguous.  If they are,
    # we need to cause an error to bubble up in order to alert the test writer that we may not
    # be checking what they intend to check

    def name_match_fn(proc):
        return proc_name in proc.process_details['name']

    def cmd_match_fn(proc):
        if cmd_args is None:
            return True
        elif cmd_args is NO_CMD_ARGS:
            return len(proc.process_details['cmd']) == 1
        else:
            return cmd_args in proc.process_details['cmd'][1:]

    unique_procs = proc_output.processes()
    matching_procs = [proc for proc in unique_procs if name_match_fn(proc) and cmd_match_fn(proc)]

    if len(matching_procs) == 0:
        proc_names = ', '.join(sorted([_proc_to_name_and_args(proc) for proc in unique_procs]))

        raise Exception(
            "Did not find any processes matching name '{}' and args '{}'. Procs: {}".format(
                proc_name,
                cmd_args,
                proc_names
            )
        )
    elif strict_proc_matching and len(matching_procs) > 1:
        proc_names = ', '.join(sorted([_proc_to_name_and_args(proc) for proc in matching_procs]))
        raise Exception(
            "Found multiple processes matching name '{}' and cmd_args '{}'. Procs: {}".format(
                proc_name,
                cmd_args,
                proc_names
            )
        )

    for proc in matching_procs:  # Nominally just one matching proc
        for output in proc_output[proc]:
            if msg in output.text.decode('ascii'):
                return
    else:
        assert False, "Did not find '{}' in output for process {} {}".format(
            msg,
            proc_name,
            cmd_args
        )


def assertInStdout(proc_output,
                   msg,
                   proc,
                   cmd_args=None,
                   *,
                   strict_proc_matching=True):
    """Assert that 'msg' was found in the standard out of a process.

    :param proc_output: The process output captured by apex_launchtest.  This is usually injected
    into test cases as self._proc_output
    :type proc_output: An apex_launchtest.IoHandler

    :param msg: The message to search for
    :type msg: string

    :param proc: The process whose output will be searched
    :type proc: A string (search by process name) or a launch.actions.ExecuteProcess object

    :param cmd_args: Optional.  If 'proc' is a string, cmd_args will be used to disambiguate
    processes with the same name.  Pass apex_launchtest.asserts.NO_CMD_ARGS to match a proc without
    command arguments
    :type cmd_args: string

    :param strict_proc_matching: Optional (default True), If proc is a string and the combination
    of proc and cmd_args matches multiple processes, then strict_proc_matching=True will raise
    an error.
    :type strict_proc_matching: bool
    """
    # Depending on the type of 'proc' we're going to dispatch this a little differently
    if isinstance(proc, launch.actions.ExecuteProcess):
        _assertInStdoutByProcessAction(
            proc_output,
            msg,
            proc
        )
    elif isinstance(proc, str):
        _assertInStdoutByStringProcessName(
            proc_output,
            msg,
            proc,
            cmd_args,
            strict_proc_matching
        )
    else:
        raise TypeError(
            "proc argument must be 'ExecuteProcess' or 'str' not {}".format(type(proc))
        )
