# Copyright 2019 Open Source Robotics Foundation, Inc.
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

"""Module for OnProcessStart class."""

import collections.abc
from typing import Callable
from typing import cast
from typing import List  # noqa
from typing import Optional
from typing import Text
from typing import TYPE_CHECKING
from typing import Union

from ..event import Event
from ..event_handler import BaseEventHandler
from ..events.process import ProcessStarted
from ..launch_context import LaunchContext
from ..launch_description_entity import LaunchDescriptionEntity
from ..some_actions_type import SomeActionsType

if TYPE_CHECKING:
    from ..actions import ExecuteProcess  # noqa: F401


class OnProcessStart(BaseEventHandler):
    """
    Convenience class for handling a process started event.

    It may be configured to only handle the starting of a specific action,
    or to handle all started processes.
    """

    def __init__(
        self,
        *,
        target_action: 'ExecuteProcess' = None,
        on_start: Union[SomeActionsType,
                        Callable[[ProcessStarted, LaunchContext], Optional[SomeActionsType]]],
        **kwargs
    ) -> None:
        """Create an OnProcessStart event handler."""
        from ..actions import ExecuteProcess  # noqa
        if not isinstance(target_action, (ExecuteProcess, type(None))):
            raise TypeError("OnProcessStart requires an 'ExecuteProcess' action as the target")
        super().__init__(
            matcher=(
                lambda event: (
                    isinstance(event, ProcessStarted) and (
                        target_action is None or
                        event.action == target_action
                    )
                )
            ),
            **kwargs,
        )
        self.__target_action = target_action
        self.__actions_on_start = []  # type: List[LaunchDescriptionEntity]
        # TODO(sloretz) check that callable matches correct signature
        if callable(on_start):
            # on_start is a function or lambda that returns actions
            self.__on_start = on_start
        elif isinstance(on_start, collections.abc.Iterable):
            for entity in on_start:
                if not isinstance(entity, LaunchDescriptionEntity):
                    raise ValueError(
                        "expected all items in 'on_start' iterable to be of type "
                        "'LaunchDescriptionEntity' but got '{}'".format(type(entity)))
            self.__actions_on_start = list(on_start)  # Outside list is to ensure type is List
        elif isinstance(on_start, LaunchDescriptionEntity):
            self.__actions_on_start = [on_start]
        else:
            raise TypeError('on_start with type {} not allowed'.format(repr(on_start)))

    def handle(self, event: Event, context: LaunchContext) -> Optional[SomeActionsType]:
        """Handle the given event."""
        super().handle(event, context)

        if self.__actions_on_start:
            return self.__actions_on_start
        return self.__on_start(cast(ProcessStarted, event), context)

    @property
    def handler_description(self) -> Text:
        """Return the string description of the handler."""
        if self.__actions_on_start:
            return '<actions>'
        return '{}'.format(self.__on_start)

    @property
    def matcher_description(self) -> Text:
        """Return the string description of the matcher."""
        if self.__target_action is None:
            return 'event == ProcessStarted'
        return 'event == ProcessStarted and event.action == ExecuteProcess({})'.format(
            hex(id(self.__target_action))
        )
