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

"""Module for signal management functionality."""

import asyncio
import os
import signal
import socket


from typing import Callable
from typing import Optional
from typing import Union


class AsyncSafeSignalManager:
    """
    A context manager class for asynchronous handling of signals.

    Similar in purpose to :func:`asyncio.loop.add_signal_handler` but
    not limited to Unix platforms.

    Signal handlers can be registered at any time with a given manager.
    These will become active for the extent of said manager context.
    Unlike regular signal handlers, asynchronous signals handlers
    can safely interact with their event loop.

    The same manager can be used multiple consecutive times and even
    be nested with other managers, as these are independent from each
    other i.e. managers do not override each other's handlers.

    If used outside of the main thread, a ValueError is raised.

    The underlying mechanism is built around :func:`signal.set_wakeup_fd`
    so as to not interfere with regular handlers installed via
    :func:`signal.signal`.
    All signals received are forwarded to the previously setup file
    descriptor, if any.
    """

    def __init__(
        self,
        loop: asyncio.AbstractEventLoop
    ):
        """
        Instantiate manager.

        :param loop: event loop that will handle the signals.
        """
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

    def handle(
        self,
        signum: Union[signal.Signals, int],
        handler: Optional[Callable[[int], None]],
    ) -> Optional[Callable[[int], None]]:
        """
        Register a callback for asynchronous handling of a given signal.

        :param signum: number of the signal to be handled
        :param handler: callback taking a signal number
          as its sole argument, or None
        :return: previous handler if any, otherwise None
        """
        signum = signal.Signals(signum)
        if handler is not None:
            if not callable(handler):
                raise ValueError('signal handler must be a callable')
            old_handler = self.__handlers.get(signum, None)
            self.__handlers[signum] = handler
        else:
            old_handler = self.__handlers.pop(signum, None)
        return old_handler
