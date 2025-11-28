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

"""Module for the ReplaceEnvironmentVariables action."""

from typing import List
from typing import Mapping
from typing import Text

from ..action import Action
from ..frontend import Entity
from ..frontend import expose_action
from ..frontend import Parser
from ..launch_context import LaunchContext
from ..utilities import normalize_to_list_of_substitutions
from ..utilities import perform_substitutions


@expose_action('rep_env')
class ReplaceEnvironmentVariables(Action):
    """
    Action that replaces the environment variables in the current context.

    The previous state can be saved by pushing the stack with the
    :py:class:`launch.actions.PushEnvironment` action.
    And can be restored by popping the stack with the
    :py:class:`launch.actions.PopEnvironment` action.
    """

    def __init__(self, environment: Mapping[Text, Text] = {}, **kwargs) -> None:
        """Create a ReplaceEnvironmentVariables action."""
        super().__init__(**kwargs)
        self.__environment = environment

    @classmethod
    def parse(
        cls,
        entity: Entity,
        parser: Parser
    ):
        """Return the `ReplaceEnvironmentVariables` action and kwargs for constructing it."""
        _, kwargs = super().parse(entity, parser)
        env = entity.get_attr('env', data_type=List[Entity], optional=True)

        if env is not None:
            kwargs['environment'] = {
                tuple(parser.parse_substitution(e.get_attr('name'))):
                parser.parse_substitution(e.get_attr('value')) for e in env
            }
            for e in env:
                e.assert_entity_completely_parsed()
        return cls, kwargs

    @property
    def environment(self):
        """Getter for environment."""
        return self.__environment

    def execute(self, context: LaunchContext):
        """Execute the action."""
        evaluated_environment = {}
        for k, v in self.__environment.items():
            evaluated_k = perform_substitutions(context, normalize_to_list_of_substitutions(k))
            evaluated_v = perform_substitutions(context, normalize_to_list_of_substitutions(v))
            evaluated_environment[evaluated_k] = evaluated_v

        context._replace_environment(evaluated_environment)
