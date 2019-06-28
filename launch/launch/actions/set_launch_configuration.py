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

"""Module for the SetLaunchConfiguration action."""

from typing import List

from ..action import Action
from ..launch_context import LaunchContext
from ..launch_frontend import Entity
from ..launch_frontend import expose_action
from ..launch_frontend import Parser
from ..some_substitutions_type import SomeSubstitutionsType
from ..substitution import Substitution
from ..utilities import normalize_to_list_of_substitutions
from ..utilities import perform_substitutions


@expose_action('let')
class SetLaunchConfiguration(Action):
    """
    Action that sets a launch configuration by name.

    Launch configurations can be accessed by the LaunchConfiguration
    substitution and are accessible after being set, even in included
    LaunchDescription's, but can be scoped with groups.
    """

    def __init__(
        self,
        name: SomeSubstitutionsType,
        value: SomeSubstitutionsType,
        **kwargs
    ) -> None:
        """Constructor."""
        super().__init__(**kwargs)
        self.__name = normalize_to_list_of_substitutions(name)
        self.__value = normalize_to_list_of_substitutions(value)

    @staticmethod
    def parse(entity: Entity, parser: Parser):
        """Return `SetLaunchConfiguration` action and kwargs for constructing it."""
        name = parser.parse_substitution(entity.get_attr('name'))
        value = parser.parse_substitution(entity.get_attr('value'))
        _, kwargs = super(SetLaunchConfiguration, SetLaunchConfiguration).parse(entity, parser)
        kwargs['name'] = name
        kwargs['value'] = value
        return SetLaunchConfiguration, kwargs

    @property
    def name(self) -> List[Substitution]:
        """Getter for self.__name."""
        return self.__name

    @property
    def value(self) -> List[Substitution]:
        """Getter for self.__value."""
        return self.__value

    def execute(self, context: LaunchContext):
        """Execute the action."""
        context.launch_configurations[perform_substitutions(context, self.name)] = \
            perform_substitutions(context, self.value)
