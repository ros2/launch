# Copyright 2021 Open Source Robotics Foundation, Inc.
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

"""Module for OnProcessEventBase class."""

import collections.abc
from typing import Callable
from typing import cast
from typing import List  # noqa
from typing import Optional
from typing import Text
from typing import Type
from typing import TYPE_CHECKING
from typing import Union

from launch.events.process.running_process_event import RunningProcessEvent

from ..event import Event
from ..event_handler import BaseEventHandler
from ..launch_context import LaunchContext
from ..launch_description_entity import LaunchDescriptionEntity
from ..some_actions_type import SomeActionsType

if TYPE_CHECKING:
    from ..actions import ExecuteProcess  # noqa: F401


class OnProcessEventBase(BaseEventHandler):
    """Base event handler for handlers targeting a subtype of RunningProcessEvent."""

    def __init__(
        self,
        *,
        process_matcher: Optional[Union[Callable[['ExecuteProcess'], bool], 'ExecuteProcess']],
        on_event: Union[
            SomeActionsType,
            Callable[[RunningProcessEvent, LaunchContext], Optional[SomeActionsType]]
        ],
        target_event_cls: Type[RunningProcessEvent],
        **kwargs
    ) -> None:
        """
        Construct a `OnProcessEventBase` instance.

        :param process_matcher: `ExecuteProcess` instance or callable to filter events
            from which proces/processes to handle.
        :param on_event: Action to be done to handle the event.
        :param target_event_cls: A subclass of `RunningProcessEvent`, indicating which events
            should be handled.
        """
        from ..actions import ExecuteProcess  # noqa
        if not issubclass(target_event_cls, RunningProcessEvent):
            raise TypeError("'target_event_cls' must be a subclass of 'RunningProcessEvent'")
        if (
            not isinstance(process_matcher, (ExecuteProcess, type(None)))
            and not callable(process_matcher)
        ):
            raise TypeError("process_matcher must be an 'ExecuteProcess' instance or a callable")
        self.__target_event_cls = target_event_cls
        self.__process_matcher = process_matcher

        def event_matcher(event):
            if not isinstance(event, target_event_cls):
                return False
            if callable(process_matcher):
                return process_matcher(event.action)
            if isinstance(process_matcher, ExecuteProcess):
                return event.action is process_matcher
            assert process_matcher is None
            return True
        super().__init__(matcher=event_matcher, **kwargs)
        self.__actions_on_event: List[LaunchDescriptionEntity] = []
        # TODO(wjwwood) check that it is not only callable, but also a callable that matches
        # the correct signature for a handler in this case
        if callable(on_event):
            # Then on_exit is a function or lambda, so we can just call it, but
            # we don't put anything in self.__actions_on_event because we cannot
            # know what the function will return.
            self.__on_event = on_event
        else:
            # Otherwise, setup self.__actions_on_event
            if isinstance(on_event, collections.abc.Iterable):
                for entity in on_event:
                    if not isinstance(entity, LaunchDescriptionEntity):
                        raise ValueError(
                            "expected all items in 'on_event' iterable to be of type "
                            "'LaunchDescriptionEntity' but got '{}'".format(type(entity)))
                self.__actions_on_event = list(on_event)  # Outside list is to ensure type is List
            else:
                self.__actions_on_event = [on_event]

    def handle(self, event: Event, context: LaunchContext) -> Optional[SomeActionsType]:
        """Handle the given event."""
        super().handle(event, context)

        if self.__actions_on_event:
            return self.__actions_on_event
        return self.__on_event(cast(RunningProcessEvent, event), context)

    @property
    def handler_description(self) -> Text:
        """Return the string description of the handler."""
        # TODO(jacobperron): revisit how to describe known actions that are passed in.
        #                    It would be nice if the parent class could output their description
        #                    via the 'entities' property.
        if self.__actions_on_event:
            return '<actions>'
        return '{}'.format(self.__on_event)

    @property
    def matcher_description(self) -> Text:
        """Return the string description of the matcher."""
        if self.__target_action is None:
            return f'event == {self.__target_event_cls.__name__}'
        return (
            f'event == {self.__target_event_cls.__name__} and'
            f' {self.__process_matcher.__name__}(event.action)'
        )
