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

from .asserts.assert_output import assertInStdout


class IoHandler:
    """
    Holds stdout captured from running processes.

    This class provides helper methods to enumerate the captured IO by individual processes
    """

    def __init__(self):
        self._sequence_list = []  # A time-ordered list of IO from all processes
        self._process_name_dict = {}  # A dict of time ordered lists of IO key'd by the process

    def append(self, process_io):
        self._sequence_list.append(process_io)

        if process_io.process_name not in self._process_name_dict:
            self._process_name_dict[process_io.process_name] = []

        self._process_name_dict[process_io.process_name].append(process_io)

    def __iter__(self):
        return self._sequence_list.__iter__()

    def processes(self):
        """
        Get an iterable of unique launch.events.process.RunningProcessEvent objects.

        :returns [launch.actions.ExecuteProcess]:
        """
        return map(
            lambda x: x.action,
            [self._process_name_dict[name][0] for name in self._process_name_dict]
        )

    def process_names(self):
        """
        Get the name of all unique processes that generated IO.

        :returns [string]:
        """
        return self._process_name_dict.keys()

    def __getitem__(self, key):
        """
        Get the output for a given process or process name.

        :param key: The process to get the output for
        :type key: String, or launch.actions.ExecuteProcess
        """
        if isinstance(key, str):
            return list(self._process_name_dict[key])
        else:
            return list(self._process_name_dict[key.process_details['name']])


class ActiveIoHandler(IoHandler):
    """
    Holds stdout captured from running processes.

    The ActiveIoHandler is meant to be used when capturing is still in progress and provides
    additional synchronization, as well as methods to wait on incoming IO
    """

    def __init__(self):
        self._sync_lock = threading.Condition()
        # Deliberately not calling the 'super' constructor here.  We're building this class
        # by composition so we can still give out the unsynchronized version
        self._io_handler = IoHandler()

    def append(self, process_io):
        with self._sync_lock:
            self._io_handler.append(process_io)
            self._sync_lock.notify()

    def __iter__(self):
        with self._sync_lock:
            return list(self._io_handler).__iter__()

    def processes(self):
        """
        Get an iterable of unique launch.events.process.RunningProcessEvent objects.

        :returns [launch.actions.ExecuteProcess]:
        """
        with self._sync_lock:
            return list(self._io_handler.processes())

    def process_names(self):
        """
        Get the name of all unique processes that generated IO.

        :returns [string]:
        """
        with self._sync_lock:
            return list(self._io_handler.process_names())

    def __getitem__(self, key):
        """
        Get the output for a given process or process name.

        :param key: The process to get the output for
        :type key: String, or launch.actions.ExecuteProcess
        """
        with self._sync_lock:
            return self._io_handler[key]

    def assertWaitFor(self, msg, timeout):
        success = False

        def msg_found():
            try:
                assertInStdout(
                    self._io_handler,  # Use unsynchronized, since this is called from a lock
                    msg=msg,
                    proc="",           # Will match all process names
                    cmd_args=None,     # Will match all cmd args
                    strict_proc_matching=False
                )
                return True
            except Exception:
                # TODO (pete baughman) This is here to handle the case where
                # _assertInStdoutByStringProcessName raises an exception because no proc has
                # generated output yet.  Perhaps a more specific exception should be used and
                # then we catch the specific exception type
                return False
            except AssertionError:
                return False

        with self._sync_lock:
            # TODO(pete.baughman): Searching through all of the IO can be time consuming/wasteful.
            # We can optimize this by making a note of where we left off searching and only
            # searching new messages when we return from the wait.
            success = self._sync_lock.wait_for(
                msg_found,
                timeout=timeout
            )
        assert success, "Wait for msg '{}' timed out".format(msg)
