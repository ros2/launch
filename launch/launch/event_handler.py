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
    """Base class for event handlers, which handle events in the launch system."""

    def __init__(
        self,
        *,
        matcher: Callable[[Event], bool],
        handler: Optional[Callable[[Event, 'LaunchContext'], Optional[SomeActionsType]]]
    ) -> None:
        """Constructor."""
        self.__matcher = matcher
        self.__handler = handler

    def matches(self, event: Event) -> bool:
        """Return True if the given event should be handled by this handler."""
        return self.__matcher(event)

    def handle(self, event: Event, context: 'LaunchContext') -> Optional[SomeActionsType]:
        """Handle the given event."""
        return self.__handler(event, context) if self.__handler is not None else None
