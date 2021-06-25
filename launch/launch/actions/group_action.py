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

from .pop_launch_configurations import PopLaunchConfigurations
from .push_launch_configurations import PushLaunchConfigurations
from .reset_launch_configurations import ResetLaunchConfigurations
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
    Action that yields other actions, optionally scoping and forwarding launch configurations.

    This action is used to nest other actions without including a separate
    launch description, while also optionally having a condition (like all
    other actions), scoping launch configurations, forwarding launch
    configurations, and/or declaring launch configurations for just the
    group and its yielded actions.

    When scoped is set to True, changes to launch configurations are
    limited to the scope of actions in the group action.

    When scoped is set to True and forwarded is set to True, all existing
    launch configurations are available in the scoped context.

    When scoped is set to True and forwarded is set to False, all existing
    launch configurations are removed from the scoped context unless it is
    explicitly forwarded by adding its key to the launch_configurations
    dictionary with a None value.

    Example: All launch configurations except the one with key of 'arg1'
    will be removed from the scoped context.

    .. code-block:: python

        GroupAction(
            scoped=True,
            forwarded=False,
            launch_configurations={'arg1': None}
        )
        ...
    """

    def __init__(
        self,
        actions: Iterable[Action],
        *,
        scoped: bool = True,
        forwarding: bool = True,
        launch_configurations: Optional[Dict[SomeSubstitutionsType, SomeSubstitutionsType]] = None,
        **left_over_kwargs
    ) -> None:
        """Create a GroupAction."""
        super().__init__(**left_over_kwargs)
        self.__actions = actions
        self.__scoped = scoped
        self.__forwarding = forwarding
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
        forwarding = entity.get_attr('forwarding', data_type=bool, optional=True)
        if scoped is not None:
            kwargs['scoped'] = scoped
        if forwarding is not None:
            kwargs['forwarding'] = forwarding
        kwargs['actions'] = [parser.parse_action(e) for e in entity.children]
        return cls, kwargs

    def get_sub_entities(self) -> List[LaunchDescriptionEntity]:
        """Return subentities."""
        if self.__actions_to_return is None:
            self.__actions_to_return = list(self.__actions)
            configuration_sets = [
                SetLaunchConfiguration(k, v) for k, v in self.__launch_configurations.items()
            ]
            if self.__scoped:
                if self.__forwarding:
                    self.__actions_to_return = [
                        PushLaunchConfigurations(),
                        *configuration_sets,
                        *self.__actions_to_return,
                        PopLaunchConfigurations()
                    ]
                else:
                    self.__actions_to_return = [
                        PushLaunchConfigurations(),
                        ResetLaunchConfigurations(self.__launch_configurations),
                        *self.__actions_to_return,
                        PopLaunchConfigurations()
                    ]
            else:
                self.__actions_to_return = [
                    *configuration_sets,
                    *self.__actions_to_return
                ]
        return self.__actions_to_return

    def execute(self, context: LaunchContext) -> Optional[List[LaunchDescriptionEntity]]:
        """Execute the action."""
        return self.get_sub_entities()
