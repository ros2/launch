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

"""Module for OnProcessExit class."""

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
from ..events.process import ProcessExited
from ..launch_context import LaunchContext
from ..launch_description_entity import LaunchDescriptionEntity
from ..some_actions_type import SomeActionsType

if TYPE_CHECKING:
    from ..actions import ExecuteProcess  # noqa: F401


class OnProcessExit(BaseEventHandler):
    """
    Convenience class for handling a process exited event.

    It may be configured to only handle the exiting of a specific action,
    or to handle all exited processes.
    """

    def __init__(
        self,
        *,
        target_action: 'ExecuteProcess' = None,
        on_exit: Union[SomeActionsType,
                       Callable[[ProcessExited, LaunchContext], Optional[SomeActionsType]]],
        **kwargs
    ) -> None:
        """Create an OnProcessExit event handler."""
        from ..actions import ExecuteProcess  # noqa
        if not isinstance(target_action, (ExecuteProcess, type(None))):
            raise TypeError("OnProcessExit requires an 'ExecuteProcess' action as the target")
        super().__init__(
            matcher=(
                lambda event: (
                    isinstance(event, ProcessExited) and (
                        target_action is None or
                        event.action == target_action
                    )
                )
            ),
            **kwargs,
        )
        self.__target_action = target_action
        self.__actions_on_exit = []  # type: List[LaunchDescriptionEntity]
        # TODO(wjwwood) check that it is not only callable, but also a callable that matches
        # the correct signature for a handler in this case
        if callable(on_exit):
            # Then on_exit is a function or lambda, so we can just call it, but
            # we don't put anything in self.__actions_on_exit because we cannot
            # know what the function will return.
            self.__on_exit = on_exit
        else:
            # Otherwise, setup self.__actions_on_exit
            if isinstance(on_exit, collections.abc.Iterable):
                for entity in on_exit:
                    if not isinstance(entity, LaunchDescriptionEntity):
                        raise ValueError(
                            "expected all items in 'on_exit' iterable to be of type "
                            "'LaunchDescriptionEntity' but got '{}'".format(type(entity)))
                self.__actions_on_exit = list(on_exit)  # Outside list is to ensure type is List
            else:
                self.__actions_on_exit = [on_exit]

    def handle(self, event: Event, context: LaunchContext) -> Optional[SomeActionsType]:
        """Handle the given event."""
        super().handle(event, context)

        if self.__actions_on_exit:
            return self.__actions_on_exit
        return self.__on_exit(cast(ProcessExited, event), context)

    @property
    def handler_description(self) -> Text:
        """Return the string description of the handler."""
        # TODO(jacobperron): revisit how to describe known actions that are passed in.
        #                    It would be nice if the parent class could output their description
        #                    via the 'entities' property.
        if self.__actions_on_exit:
            return '<actions>'
        return '{}'.format(self.__on_exit)

    @property
    def matcher_description(self) -> Text:
        """Return the string description of the matcher."""
        if self.__target_action is None:
            return 'event == ProcessExited'
        return 'event == ProcessExited and event.action == ExecuteProcess({})'.format(
            hex(id(self.__target_action))
        )
