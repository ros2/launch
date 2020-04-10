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

import collections.abc
from typing import Callable
from typing import cast
from typing import List  # noqa
from typing import Optional
from typing import Text
from typing import TYPE_CHECKING
from typing import Union

from ..event import Event
from ..event_handler import EventHandler
from ..events import ExecutionComplete
from ..launch_context import LaunchContext
from ..launch_description_entity import LaunchDescriptionEntity
from ..some_actions_type import SomeActionsType

if TYPE_CHECKING:
    from .. import Action  # noqa


class OnExecutionComplete(EventHandler):
    """
    Convenience class for handling an action completion event.

    It may be configured to only handle the completion of a specific action,
    or to handle them all.
    """

    def __init__(
        self,
        *,
        target_action: Optional['Action'] = None,
        on_completion: Union[SomeActionsType, Callable[[int], Optional[SomeActionsType]]],
        **kwargs
    ) -> None:
        """Create an OnExecutionComplete event handler."""
        from ..action import Action  # noqa
        if not isinstance(target_action, (Action, type(None))):
            raise ValueError("OnExecutionComplete requires an 'Action' as the target")
        super().__init__(
            matcher=(
                lambda event: (
                    isinstance(event, ExecutionComplete) and (
                        target_action is None or
                        event.action == target_action
                    )
                )
            ),
            entities=None,
            **kwargs,
        )
        self.__target_action = target_action
        # TODO(wjwwood) check that it is not only callable, but also a callable that matches
        # the correct signature for a handler in this case
        self.__on_completion = on_completion
        self.__actions_on_completion = []  # type: List[LaunchDescriptionEntity]
        if callable(on_completion):
            # Then on_completion is a function or lambda, so we can just call it, but
            # we don't put anything in self.__actions_on_completion because we cannot
            # know what the function will return.
            pass
        else:
            # Otherwise, setup self.__actions_on_completion
            if isinstance(on_completion, collections.abc.Iterable):
                for entity in on_completion:
                    if not isinstance(entity, LaunchDescriptionEntity):
                        raise ValueError(
                            "expected all items in 'on_completion' iterable to be of type "
                            "'LaunchDescriptionEntity' but got '{}'".format(type(entity)))
                self.__actions_on_completion = list(on_completion)
            else:
                self.__actions_on_completion = [on_completion]
            # Then return it from a lambda and use that as the self.__on_completion callback.
            self.__on_completion = lambda event, context: self.__actions_on_completion

    def handle(self, event: Event, context: LaunchContext) -> Optional[SomeActionsType]:
        """Handle the given event."""
        return self.__on_completion(cast(ExecutionComplete, event), context)

    @property
    def handler_description(self) -> Text:
        """Return the string description of the handler."""
        # TODO(jacobperron): revisit how to describe known actions that are passed in.
        #                    It would be nice if the parent class could output their description
        #                    via the 'entities' property.
        if self.__actions_on_completion:
            return '<actions>'
        return '{}'.format(self.__on_completion)

    @property
    def matcher_description(self) -> Text:
        """Return the string description of the matcher."""
        if self.__target_action is None:
            return 'event == ExecutionComplete'
        return 'event == ExecutionComplete and event.action == Action({})'.format(
            hex(id(self.__target_action))
        )
