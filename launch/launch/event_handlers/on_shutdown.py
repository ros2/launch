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

"""Module for OnShutdown class."""

from typing import Callable
from typing import List
from typing import Optional
from typing import Text
from typing import Tuple
from typing import overload

from ..event_handler import EventHandler
from ..events import Shutdown
from ..launch_description_entity import LaunchDescriptionEntity
from ..some_actions_type import SomeActionsType
from ..utilities import is_a_subclass


class OnShutdown(EventHandler):
    """Convenience class for handling the launch shutdown event."""

    @overload
    def __init__(self, *, on_shutdown: SomeActionsType):
        """Overload which takes just actions."""
        ...

    @overload  # noqa: F811
    def __init__(
        self,
        *,
        on_shutdown: Callable[[Shutdown, 'LaunchContext'], Optional[SomeActionsType]]
    ):
        """Overload which takes a callable to handle the shutdown."""
        ...

    def __init__(self, *, on_shutdown):  # noqa: F811
        """Constructor."""
        super().__init__(
            matcher=lambda event: is_a_subclass(event, Shutdown),
            handler=None,  # noop
        )
        # TODO(wjwwood) check that it is not only callable, but also a callable that matches
        # the correct signature for a handler in this case
        self.__on_shutdown = on_shutdown
        if not callable(on_shutdown):
            self.__on_shutdown = (lambda event, context: on_shutdown)

    def handle(self, event: Shutdown, context: 'LaunchContext') -> Optional[SomeActionsType]:
        """Handle the given event."""
        return self.__on_shutdown(event, context)

    def describe(self) -> Tuple[Text, List[LaunchDescriptionEntity]]:
        """Return the description list with 0 being a string, and then LaunchDescriptionEntity's."""
        return [
            "OnShutdown(matcher='{}', handler={})".format(
                self.matcher_description,
                self.__on_shutdown),
            [],
        ]

    @property
    def matcher_description(self):
        """Return the string description of the matcher."""
        return 'event issubclass of launch.events.Shutdown'
