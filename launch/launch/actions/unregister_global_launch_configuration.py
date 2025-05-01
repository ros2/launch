# Copyright 2025 Open Source Robotics Foundation, Inc.
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

"""Module for the UnregisterGlobalLaunchConfiguration."""

from typing import List

from ..action import Action
from ..frontend import Entity
from ..frontend import expose_action
from ..frontend import Parser
from ..launch_context import LaunchContext
from ..some_substitutions_type import SomeSubstitutionsType
from ..substitution import Substitution
from ..utilities import normalize_to_list_of_substitutions
from ..utilities import unregister_global


@expose_action('unregister-global')
@expose_action('unregister_global')
class UnregisterGlobalLaunchConfiguration(Action):
    """Action that add a launch configuration to the globals set by name."""

    def __init__(self, name: SomeSubstitutionsType, **kwargs) -> None:
        """Create a UnregisterGlobalLaunchConfiguration action."""
        super().__init__(**kwargs)
        self.__name = normalize_to_list_of_substitutions(name)

    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        """Return `UnregisterGlobalLaunchConfiguration` action and kwargs for constructing it."""
        name = parser.parse_substitution(entity.get_attr('name'))
        _, kwargs = super().parse(entity, parser)
        kwargs['name'] = name
        return cls, kwargs

    @property
    def name(self) -> List[Substitution]:
        """Getter for self.__name."""
        return self.__name

    def execute(self, context: LaunchContext):
        """Execute the action."""
        unregister_global(context, self.name)
