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

"""Module for EventHandler class."""

from typing import Callable
from typing import Optional

from .event import Event
from .some_actions_type import SomeActionsType

if False:
    # imports here would cause loops, but are only used as forward-references for type-checking
    from .launch_context import LaunchContext  # noqa


class EventHandler:
    """
    Base class for event handlers, which handle events in the launch system.

    Entities yielded by the event handler can access the event being handled
    via the context's locals, e.g. `context.locals.event`
    As another example, getting the name of the event as a Substitution:
    `launch.substitutions.LocalSubstitution('event.name')`.
    """

    def __init__(
        self,
        *,
        matcher: Callable[[Event], bool],
        entities: Optional[SomeActionsType] = None,
        handle_once: bool = False
    ) -> None:
        """
        Constructor.

        :param: matcher is a callable that takes an event and returns True if
            the event should be handled by this event handler, False otherwise.
        :param: entities is an LaunchDescriptionEntity or list of them, and is
            returned by handle() unconditionally if matcher returns True.
        :param: handle_once is a flag that, if True, unregisters this EventHandler
            after being handled once.
        """
        self.__matcher = matcher
        self.__entities = entities
        self.handle_once = handle_once

    @property
    def entities(self):
        """Getter for entities."""
        return self.__entities

    @property
    def handle_once(self):
        """Getter for handle_once flag."""
        return self.__handle_once

    @handle_once.setter
    def handle_once(self, value):
        """Setter for handle_once flag."""
        if not isinstance(value, bool):
            raise TypeError(
                'handle_once expects type "bool", but received {} instead.'.format(type(value))
            )
        self.__handle_once = value

    # TODO(wjwwood): setup standard interface for describing event handlers

    def matches(self, event: Event) -> bool:
        """Return True if the given event should be handled by this event handler."""
        return self.__matcher(event)

    def handle(self, event: Event, context: 'LaunchContext') -> Optional[SomeActionsType]:
        """Handle the given event."""
        context.extend_locals({'event': event})
        if self.handle_once:
            context.unregister_event_handler(self)
        return self.__entities
