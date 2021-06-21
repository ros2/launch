# Copyright 2021 Open Source Robotics Foundation, Inc.
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

"""Module for the ClearLaunchConfigurations action."""

from typing import List
from typing import Optional

from ..action import Action
from ..launch_context import LaunchContext
from ..some_substitutions_type import SomeSubstitutionsType

from ..utilities import perform_substitutions


class ClearLaunchConfigurations(Action):
    """Action that clears the launch configurations from the context."""

    def __init__(
        self,
        forwarded_configurations: Optional[List[SomeSubstitutionsType]] = None,
        **kwargs
    ) -> None:
        """Create a ClearLaunchConfigurations action."""
        super().__init__(**kwargs)
        self.__forwarded_configurations = forwarded_configurations

    def execute(self, context: LaunchContext):
        """Execute the action."""
        if self.__forwarded_configurations is not None:
            forwarded_keys = []
            for cfg in self.__forwarded_configurations:
                key = perform_substitutions(context, cfg.variable_name)
                forwarded_keys.append(key)

            delete = [key for key in context.launch_configurations if key not in forwarded_keys]

            for key in delete:
                del context.launch_configurations[key]

        else:
            context.launch_configurations.clear()
