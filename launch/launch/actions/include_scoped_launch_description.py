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

"""Module for the ScopedIncludeLaunchDescription action."""

# from typing import override # Available starting from Python3.12
from typing import Dict
from typing import List
from typing import Text

from .include_launch_description import IncludeLaunchDescription
from .pop_environment import PopEnvironment
from .pop_launch_configurations import PopLaunchConfigurations
from .push_environment import PushEnvironment
from .push_launch_configurations import PushLaunchConfigurations
from .reset_environment import ResetEnvironment
from .reset_launch_configurations import ResetLaunchConfigurations
from .set_launch_configuration import SetLaunchConfiguration
from ..frontend import expose_action
from ..launch_context import LaunchContext
from ..launch_description_entity import LaunchDescriptionEntity
from ..some_substitutions_type import SomeSubstitutionsType
from ..utilities import normalize_to_list_of_substitutions
from ..utilities import perform_substitutions


@expose_action('scoped_include')
class ScopedIncludeLaunchDescription(IncludeLaunchDescription):
    # TODO(SuperJappie08) Propper Documentation

    # NOTE(SuperJappie08) __init__ is not required since the function signature will be the same
    # However maybe it is interresting for documentation purposes

    def get_sub_entities(self):
        """Get subentities."""
        # ret = super().get_sub_entities()
        # TODO(SuperJappie08)? Do these internals need to be hidden?
        # print(self.launch_arguments)
        return [
            PushLaunchConfigurations(),
            PushEnvironment(),
            ResetEnvironment(),
            # NOTE(SuperJappie08) Need weird remap, since AnySubstitution type can be a List which
            #   is not Hashable.
            # ResetLaunchConfigurations({k: v for k, v in self.launch_arguments}),
            # *ret,
            ResetLaunchConfigurations(),
            *[SetLaunchConfiguration(k, v) for k, v in self.launch_arguments],
            *super().get_sub_entities(),
            PopEnvironment(),
            PopLaunchConfigurations(),
        ]

    def execute(self, context: LaunchContext) -> List[LaunchDescriptionEntity]:
        """Execute the action."""
        evaluated_configurations: Dict[SomeSubstitutionsType, SomeSubstitutionsType] = {}

        for name in getattr(context.locals, 'globals', set()):
            expanded_name = perform_substitutions(
                context,
                normalize_to_list_of_substitutions(name)
            )

            if expanded_name in context.launch_configurations:
                evaluated_configurations[expanded_name] = \
                    context.launch_configurations[expanded_name]

        for k, v in self.launch_arguments:
            # Perform substitutions, since the required launch configurations might not be
            # available in the inner scope.
            evaluated_k = perform_substitutions(context, normalize_to_list_of_substitutions(k))
            evaluated_v = perform_substitutions(context, normalize_to_list_of_substitutions(v))
            evaluated_configurations[evaluated_k] = evaluated_v

        return [
            PushLaunchConfigurations(),
            PushEnvironment(),
            ResetEnvironment(),
            ResetLaunchConfigurations(evaluated_configurations),
            IncludeLaunchDescription(
                                     launch_description_source=self.launch_description_source,
                                     launch_arguments=self.launch_arguments,
                                     condition=self.condition),
            PopEnvironment(),
            PopLaunchConfigurations()
        ]

    def __repr__(self) -> Text:
        """Return a description of this ScopedIncludeLaunchDescription as a string."""
        return f'ScopedIncludeLaunchDescription({self.launch_description_source.location})'
