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
        # FIXME(SuperJappie08): Temporary Test to see if behavior works out

        # print(self.launch_description_source.get_launch_description(context).entities)
        # FIXME(SuperJappie08): This breaks the PushRosNameSpace Action (Cannot push a namespace in)

        evaluated_configurations = {}
        for k, v in self.launch_arguments:
            evaluated_k = perform_substitutions(context, normalize_to_list_of_substitutions(k))
            evaluated_v = perform_substitutions(context, normalize_to_list_of_substitutions(v))
            evaluated_configurations[evaluated_k] = evaluated_v

        return [
            PushLaunchConfigurations(),
            PushEnvironment(),
            ResetEnvironment(),
            ResetLaunchConfigurations(evaluated_configurations),
            # Does this reset do anything?
            # ResetLaunchConfigurations(evaluated_configurations),
            # *super().execute(context),
            IncludeLaunchDescription(
                                     launch_description_source=self.launch_description_source,
                                     launch_arguments=self.launch_arguments,
                                     condition=self.condition),
            PopEnvironment(),
            PopLaunchConfigurations()
        ]
        # NOTE(SuperJappie08) Originally this returend something based on the used actions
        #   However after further consideration the context might not be correct that way.
        context._push_launch_configurations()
        context._push_environment()

        context._reset_environment()

        # Reset Launch Configuration
        evaluated_configurations = {}
        for k, v in self.launch_arguments:
            evaluated_k = perform_substitutions(context, normalize_to_list_of_substitutions(k))
            evaluated_v = perform_substitutions(context, normalize_to_list_of_substitutions(v))
            evaluated_configurations[evaluated_k] = evaluated_v

        context.launch_configurations.clear()
        context.launch_configurations.update(evaluated_configurations)

        launch_description = self.launch_description_source.get_launch_description(context)

        # If the location does not exist, then it's likely set to '<script>' or something.
        context.extend_locals({
            'current_launch_file_path': self._get_launch_file(),
        })
        context.extend_locals({
            'current_launch_file_directory': self._get_launch_file_directory(),
        })

        # Do best effort checking to see if non-optional, non-default declared arguments
        # are being satisfied.
        my_argument_names = [
            perform_substitutions(context, normalize_to_list_of_substitutions(arg_name))
            for arg_name, arg_value in self.launch_arguments
        ]
        try:
            declared_launch_arguments = (
                launch_description.get_launch_arguments_with_include_launch_description_actions())
        except Exception as exc:
            if hasattr(exc, 'add_note'):
                exc.add_note(f'while executing {self.describe()}')  # type: ignore
            raise
        for argument, ild_actions in declared_launch_arguments:
            if argument._conditionally_included or argument.default_value is not None:
                continue
            argument_names = my_argument_names
            if ild_actions is not None:
                for ild_action in ild_actions:
                    argument_names.extend(ild_action._try_get_arguments_names_without_context())
            if argument.name not in argument_names:
                raise RuntimeError(
                    "Included launch description missing required argument '{}' "
                    "(description: '{}'), given: [{}]"
                    .format(argument.name, argument.description, ', '.join(argument_names))
                )

        # Create actions to set the launch arguments into the launch configurations.
        set_launch_configuration_actions = []
        for name, value in self.launch_arguments:
            set_launch_configuration_actions.append(SetLaunchConfiguration(name, value))

        # Set launch arguments as launch configurations and then include the launch description.
        return [
            *set_launch_configuration_actions,
            launch_description,
            PopEnvironment(),
            PopLaunchConfigurations(),
        ]

    def __repr__(self) -> Text:
        """Return a description of this ScopedIncludeLaunchDescription as a string."""
        return f'ScopedIncludeLaunchDescription({self.launch_description_source.location})'
