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

"""Module for SignalProcess event."""

from typing import Callable
from typing import Optional
from typing import Text

from .process_targeted_event import ProcessTargetedEvent
from ...utilities import get_signal_name

if False:
    # imports here would cause loops, but are only used as forward-references for type-checking
    from ...actions import ExecuteProcess  # noqa


class SignalProcess(ProcessTargetedEvent):
    """Event emitted when a signal should be sent to a process."""

    name = 'launch.events.process.SignalProcess'

    def __init__(
        self, *,
        signal_number: int,
        process_matcher: Callable[['ExecuteProcess'], bool]
    ) -> None:
        """
        Constructor.

        Takes an optional process matcher, which can be used to match the
        signal to a process.
        """
        super().__init__(process_matcher=process_matcher)
        self.__signal = signal_number
        self.__signal_name: Optional[Text] = None  # evaluate lazily

    @property
    def signal(self) -> int:
        """Getter for signal, it will match something from the signal module."""
        return self.__signal

    @property
    def signal_name(self) -> Text:
        """
        Getter for signal_name.

        It will be something like (e.g.) 'SIGINT', or the number if name is unknown.
        """
        if self.__signal_name is None:
            self.__signal_name = get_signal_name(self.__signal)
        return self.__signal_name
