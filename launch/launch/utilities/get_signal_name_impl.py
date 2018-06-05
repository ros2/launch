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

"""Module for the get_signal_name() utility function."""

import signal
from typing import Text

# signal names, harvested from signal.h
signal_names = [
    'SIGHUP',
    'SIGINT',
    'SIGQUIT',
    'SIGILL',
    'SIGTRAP',
    'SIGABRT',
    'SIGEMT',
    'SIGFPE',
    'SIGKILL',
    'SIGBUS',
    'SIGSEGV',
    'SIGSYS',
    'SIGPIPE',
    'SIGALRM',
    'SIGTERM',
    'SIGURG',
    'SIGSTOP',
    'SIGTSTP',
    'SIGCONT',
    'SIGCHLD',
    'SIGTTIN',
    'SIGTTOU',
    'SIGIO',
    'SIGXCPU',
    'SIGXFSZ',
    'SIGVTALRM',
    'SIGPROF',
    'SIGWINCH',
    'SIGINFO',
    'SIGUSR1',
    'SIGUSR2',
]


def get_signal_name(signal_number: int) -> Text:
    """Return the name associated with the signal number, or the number as text if unknown."""
    def matches(module_signal_name):
        module_signal = getattr(signal, module_signal_name, None)
        return module_signal is not None and module_signal == signal_number

    first_name_matched = next(name for name in signal_names if matches(name))
    return first_name_matched if first_name_matched is not None else str(signal_number)
