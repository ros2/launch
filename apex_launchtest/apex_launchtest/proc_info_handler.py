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

import threading
from launch.actions import ExecuteProcess  # noqa


class ProcInfoHandler:
    """Captures exit codes from processes when they terminate."""

    def __init__(self):
        self._proc_info = {}

    def append(self, process_info):
        self._proc_info[process_info.action] = process_info

    def __iter__(self):
        return self._proc_info.values().__iter__()

    def processes(self):
        """Get the ExecuteProcess launch actions of all recorded processes."""
        return self._proc_info.keys()

    def process_names(self):
        """Get the name of all recorded processes."""
        return map(
            lambda x: x.process_details['name'],
            self._proc_info.keys()
        )

    def __getitem__(self, key):
        """
        Get the ProcessExited event for the specified process.

        :param key: Either a string, or a launch.actions.ExecuteProcess object
        :returns launch.events.process.ProcessExited:
        """
        if isinstance(key, str):
            # Look up by process name
            for (launch_action, value) in self._proc_info.items():
                if key in launch_action.process_details['name']:
                    return value
            else:
                raise KeyError(key)
        else:
            return self._proc_info[key]


class ActiveProcInfoHandler(ProcInfoHandler):
    """Allows tests to wait on process termination before proceeding."""

    def __init__(self):
        self._sync_lock = threading.Condition()
        # Deliberately not calling the 'super' constructor here.  We're building this class
        # by composition so we can still give out the unsynchronized version
        self._proc_info_handler = ProcInfoHandler()

    def append(self, process_info):
        with self._sync_lock:
            self._proc_info_handler.append(process_info)
            self._sync_lock.notify()

    def __iter__(self):
        with self._sync_lock:
            return self._proc_info_handler.__iter__()

    def process(self):
        """
        Get the ExecuteProcess launch actions of all recorded processes.

        :returns [launch.actions.ExecuteProcess]:
        """
        with self._sync_lock:
            return list(self._proc_info_handler).processes()

    def process_names(self):
        """
        Get the name of all recorded processes.

        :returns [string]:
        """
        with self._sync_lock:
            return list(self._proc_info_handler.process_names())

    def __getitem__(self, key):
        with self._sync_lock:
            return self._proc_info_handler[key]

    def assertWaitForShutdown(
            self,
            *,
            process: 'ExecuteProcess',
            timeout):

        success = False

        with self._sync_lock:
            success = self._sync_lock.wait_for(
                lambda: process in self._proc_info_handler.processes(),
                timeout=timeout
            )

        if not success:
            if process.process_details is None:
                assert False, "Process {} was never started".format(process)
            else:
                assert False, "Timed out waiting for process '{}' to finish".format(
                    process.process_details['name']
                )
