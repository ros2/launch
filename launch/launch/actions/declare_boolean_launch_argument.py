# Copyright 2026 Metro Robots
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

"""Module for the DeclareBooleanLaunchArgument action."""

from typing import Any
from typing import Dict
from typing import Optional
from typing import Text
from typing import Tuple
from typing import Type
from typing import Union

from .declare_launch_argument import DeclareLaunchArgument
from ..frontend import Entity
from ..frontend import expose_action
from ..frontend import Parser  # noqa: F401
from ..some_substitutions_type import SomeSubstitutionsType


@expose_action('bool_arg')
class DeclareBooleanLaunchArgument(DeclareLaunchArgument):
    """
    Action that declares a new boolean launch argument.

    This is a bit of syntactic sugar for
    :py:class:`launch.actions.DeclareLaunchArgument`
    that sets the choices to be either true or false (caps-insensitive)

    .. doctest::

        >>> ld = LaunchDescription([
        ...     DeclareBooleanLaunchArgument('simple_argument'),  # default value is False
        ...     DeclareBooleanLaunchArgument('with_default_value', default_value=True),
        ...     # other actions here, ...
        ... ])

    .. code-block:: xml

        <launch>
            <bool_arg name="simple_argument"/>
            <bool_arg name="with_default_value" default_value="true"/>
        </launch>
    """

    def __init__(
        self,
        name: Text,
        *,
        default_value: Optional[Union[SomeSubstitutionsType,
                                      bool]] = None,
        **kwargs: Any
    ) -> None:
        """Create a DeclareBooleanLaunchArgument action."""
        super().__init__(
            name=name,
            default_value=(default_value if not isinstance(default_value, bool)
                           else str(default_value)),
            choices=['true', 'false', 'True', 'False', 'TRUE', 'FALSE'],
            **kwargs
        )

    @classmethod
    def parse(
        cls,
        entity: Entity,
        parser: 'Parser'
    ) -> Tuple[Type['DeclareBooleanLaunchArgument'], Dict[str, Any]]:
        """Parse `bool_arg` tag."""
        _, kwargs = super().parse(entity, parser)

        if 'choices' in kwargs:
            raise ValueError('Cannot specify choices for bool_arg')

        return cls, kwargs
