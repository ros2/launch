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
from ..utilities import normalize_to_list_of_substitutions


class ClearLaunchConfigurations(Action):
    """
    Action that clears launch configurations unless explicitly excluded.

    If launch_configurations_to_not_be_cleared is not set or an empty
    list, all launch configurations are cleared from the current context.

    If a member of launch_configurations_to_not_be_cleared matches a key
    in the existing context, it will be preserved in the current context.
    All other launch configurations will be cleared from the current.
    context.
    """

    def __init__(
        self,
        launch_configurations_to_not_be_cleared: Optional[List[SomeSubstitutionsType]] = None,
        **kwargs
    ) -> None:
        """Create a ClearLaunchConfigurations action."""
        super().__init__(**kwargs)
        if launch_configurations_to_not_be_cleared is not None:
            self.__launch_configurations_to_not_be_cleared = [
                normalize_to_list_of_substitutions(sub) for sub in
                launch_configurations_to_not_be_cleared]
        else:
            self.__launch_configurations_to_not_be_cleared = []

    def execute(self, context: LaunchContext):
        """Execute the action."""
        if len(self.__launch_configurations_to_not_be_cleared) > 0:
            saved_keys = [perform_substitutions(context, key)
                          for key in self.__launch_configurations_to_not_be_cleared]
            deleted_keys = [key for key in context.launch_configurations
                            if key not in saved_keys]
            for key in deleted_keys:
                del context.launch_configurations[key]
        else:
            context.launch_configurations.clear()
