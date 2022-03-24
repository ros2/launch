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

"""Module for the GroupAction action."""

from typing import Dict
from typing import Iterable
from typing import List
from typing import Optional

from .pop_environment import PopEnvironment
from .pop_launch_configurations import PopLaunchConfigurations
from .push_environment import PushEnvironment
from .push_launch_configurations import PushLaunchConfigurations
from .set_launch_configuration import SetLaunchConfiguration
from ..action import Action
from ..frontend import Entity
from ..frontend import expose_action
from ..frontend import Parser
from ..launch_context import LaunchContext
from ..launch_description_entity import LaunchDescriptionEntity
from ..some_substitutions_type import SomeSubstitutionsType


@expose_action('group')
class GroupAction(Action):
    """
    Action that yields other actions.

    This action is used to nest other actions without including a separate
    launch description, while also optionally having a condition (like all
    other actions), scoping launch configurations and environment variables,
    and/or declaring launch  configurations for just the group and its yielded
    actions.
    """

    def __init__(
        self,
        actions: Iterable[Action],
        *,
        scoped: bool = True,
        launch_configurations: Optional[Dict[SomeSubstitutionsType, SomeSubstitutionsType]] = None,
        **left_over_kwargs
    ) -> None:
        """Create a GroupAction."""
        super().__init__(**left_over_kwargs)
        self.__actions = actions
        self.__scoped = scoped
        if launch_configurations is not None:
            self.__launch_configurations = launch_configurations
        else:
            self.__launch_configurations = {}
        self.__actions_to_return: Optional[List] = None

    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        """Return `GroupAction` action and kwargs for constructing it."""
        _, kwargs = super().parse(entity, parser)
        scoped = entity.get_attr('scoped', data_type=bool, optional=True)
        if scoped is not None:
            kwargs['scoped'] = scoped
        kwargs['actions'] = [parser.parse_action(e) for e in entity.children]
        return cls, kwargs

    def get_sub_entities(self) -> List[LaunchDescriptionEntity]:
        """Return subentities."""
        if self.__actions_to_return is None:
            self.__actions_to_return = []  # type: List[Action]
            self.__actions_to_return += [
                SetLaunchConfiguration(k, v) for k, v in self.__launch_configurations.items()
            ]
            self.__actions_to_return += list(self.__actions)
            if self.__scoped:
                self.__actions_to_return = [
                    PushLaunchConfigurations(),
                    PushEnvironment(),
                    *self.__actions_to_return,
                    PopEnvironment(),
                    PopLaunchConfigurations()
                ]
        return self.__actions_to_return

    def execute(self, context: LaunchContext) -> Optional[List[LaunchDescriptionEntity]]:
        """Execute the action."""
        return self.get_sub_entities()
