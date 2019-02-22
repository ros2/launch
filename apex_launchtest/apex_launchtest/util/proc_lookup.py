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


class NoMatchingProcessException(Exception):
    pass


def _proc_to_name_and_args(proc):
    # proc is a launch.actions.ExecuteProcess
    return "{} {}".format(
        proc.process_details['name'],
        " ".join(proc.process_details['cmd'][1:])
    )


def _str_name_to_process(info_obj, proc_name, cmd_args):

    def name_match_fn(proc):
        return proc_name in proc.process_details['name']

    def cmd_match_fn(proc):
        if cmd_args is None:
            return True
        elif cmd_args is NO_CMD_ARGS:
            return len(proc.process_details['cmd']) == 1
        else:
            return cmd_args in proc.process_details['cmd'][1:]

    matches = [proc for proc in info_obj.processes()
               if name_match_fn(proc) and cmd_match_fn(proc)]

    return matches


def resolveProcesses(info_obj, *, proc=None, cmd_args=None, strict_proc_matching=True):
    """
    Resolve a process name and cmd arguments to one or more launch.actions.ExecuteProcess.

    :param info_obj: a ProcInfoHandler or an IoHandler that contains processes that could match

    :returns: A list of matching processes
    """
    if proc is None:
        # We want to search all processes
        all_procs = info_obj.processes()
        if len(all_procs) == 0:
            raise NoMatchingProcessException("No data recorded for any process")
        return all_procs

    if isinstance(proc, launch.actions.ExecuteProcess):
        # We want to search a specific process
        if proc in info_obj.processes():
            return [proc]
        else:
            raise NoMatchingProcessException(
                "No data recorded for proc {}".format(_proc_to_name_and_args(proc))
            )

    elif isinstance(proc, str):
        # We want to search one (or more) processes that match a particular string.  The "or more"
        # part is controlled by the strict_proc_matching argument
        matches = _str_name_to_process(info_obj, proc, cmd_args)
        if len(matches) == 0:
            names = ', '.join(sorted([_proc_to_name_and_args(p) for p in info_obj.processes()]))

            raise NoMatchingProcessException(
                "Did not find any processes matching name '{}' and args '{}'. Procs: {}".format(
                    proc,
                    cmd_args,
                    names
                )
            )

        if strict_proc_matching and len(matches) > 1:
            names = ', '.join(sorted([_proc_to_name_and_args(p) for p in info_obj.processes()]))
            raise Exception(
                "Found multiple processes matching name '{}' and cmd_args '{}'. Procs: {}".format(
                    proc,
                    cmd_args,
                    names
                )
            )
        return list(matches)

    else:
        # Invalid argument passed for 'proc'
        raise TypeError(
            "proc argument must be 'ExecuteProcess' or 'str' not {}".format(type(proc))
        )
