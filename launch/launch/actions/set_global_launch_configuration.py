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

"""Module for the SetGlobalLaunchConfiguration action."""

from .set_launch_configuration import SetLaunchConfiguration
from ..frontend import expose_action
from ..launch_context import LaunchContext
from ..utilities import register_global


@expose_action('global')
class SetGlobalLaunchConfiguration(SetLaunchConfiguration):
    """
    Action that sets a global launch configuration by name.

    Launch configurations can be accessed by the LaunchConfiguration
    substitution and are accessible after being set, even in included
    LaunchDescription's, but can be scoped with groups.

    A global launch configuration is registered in the globals group
    to allow to in scoped included launch descriptions.
    """

    def __init__(
        self,
        **kwargs
    ) -> None:
        """Create a SetGlobalLaunchConfiguration action."""
        super().__init__(**kwargs)

    def execute(self, context: LaunchContext):
        """Execute the action."""
        register_global(context, self.name)

        super().execute(context)
