# Copyright 2018 Open Source Robotics Foundation, Inc.
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

"""Module for the signal management functionality."""

import os
import socket
import signal


class AsyncSafeSignalManager:

    def __init__(self, loop):
        self.__loop = loop
        self.__handlers = {}
        self.__prev_wsock_fd = -1
        self.__wsock, self.__rsock = socket.socketpair()
        self.__wsock.setblocking(False)
        self.__rsock.setblocking(False)

    def __enter__(self):
        self.__loop.add_reader(self.__rsock.fileno(), self.__handle_signal)
        self.__prev_wsock_fd = signal.set_wakeup_fd(self.__wsock.fileno())
        return self

    def __exit__(self, type_, value, traceback):
        assert self.__wsock.fileno() == signal.set_wakeup_fd(self.__prev_wsock_fd)
        self.__loop.remove_reader(self.__rsock.fileno())

    def __handle_signal(self):
        while True:
            try:
                data = self.__rsock.recv(4096)
                if not data:
                    break
                for signum in data:
                    if signum not in self.__handlers:
                        continue
                    self.__handlers[signum](signum)
                if self.__prev_wsock_fd != -1:
                    os.write(self.__prev_wsock_fd, data)
            except InterruptedError:
                continue
            except BlockingIOError:
                break

    def handle(self, signum, handler):
        if signum not in signal.Signals.__members__.values():
            raise ValueError('{} is not a signal number'.format(signum))
        if not callable(handler):
            raise ValueError('signal handler must be callable')
        old_handler = self.__handlers.get(signum, None)
        self.__handlers[signum] = handler
        return old_handler
