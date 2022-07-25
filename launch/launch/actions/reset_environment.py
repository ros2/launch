# Copyright 2022 Open Source Robotics Foundation, Inc.
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

"""Module for the ResetEnvironment action."""

from ..action import Action
from ..frontend import Entity
from ..frontend import expose_action
from ..frontend import Parser
from ..launch_context import LaunchContext


@expose_action('reset_env')
class ResetEnvironment(Action):
    """
    Action that resets the environment in the current context.

    Clears any changes made by previous actions to the context environment.
    The environment is reset to the state it was in when the context was created,
    i.e. the contents of ``os.environ``.
    """

    def __init__(self, **kwargs) -> None:
        """Create a ResetEnvironment action."""
        super().__init__(**kwargs)

    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        """Return ``ResetEnvironment`` action and kwargs for constructing it."""
        _, kwargs = super().parse(entity, parser)
        return cls, kwargs

    def execute(self, context: LaunchContext):
        """Execute the action."""
        context._reset_environment()
