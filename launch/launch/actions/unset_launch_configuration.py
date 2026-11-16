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

"""Module for the UnsetLaunchConfiguration action."""

from typing import Any
from typing import Dict
from typing import List
from typing import Tuple
from typing import Type

from ..action import Action
from ..frontend import Entity
from ..frontend import expose_action
from ..frontend import Parser
from ..launch_context import LaunchContext
from ..some_substitutions_type import SomeSubstitutionsType
from ..substitution import Substitution
from ..utilities import normalize_to_list_of_substitutions
from ..utilities import perform_substitutions


@expose_action('unset')
class UnsetLaunchConfiguration(Action):
    """
    Action that unsets a launch configuration by name.

    If the given launch configuration name is not set already then nothing
    happens.

    /sa :py:class:`launch.actions.SetLaunchConfiguration`
    """

    def __init__(
        self,
        name: SomeSubstitutionsType,
        **kwargs: Any
    ) -> None:
        """Create an UnsetLaunchConfiguration action."""
        super().__init__(**kwargs)
        self.__name = normalize_to_list_of_substitutions(name)

    @classmethod
    def parse(cls, entity: Entity, parser: Parser
              ) -> Tuple[Type['UnsetLaunchConfiguration'], Dict[str, Any]]:
        """Return ``UnsetLaunchConfiguration`` action and kwargs for constructing it."""
        _, kwargs = super().parse(entity, parser)
        kwargs['name'] = parser.parse_substitution(entity.get_attr('name'))
        return cls, kwargs

    @property
    def name(self) -> List[Substitution]:
        """Getter for self.__name."""
        return self.__name

    def execute(self, context: LaunchContext) -> None:
        """Execute the action."""
        key = perform_substitutions(context, self.name)
        if key in context.launch_configurations:
            del context.launch_configurations[key]
