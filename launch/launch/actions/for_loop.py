# Copyright 2024 Open Source Robotics Foundation, Inc.
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

"""Module for the ForLoop action."""

from copy import deepcopy
from typing import Callable
from typing import List
from typing import Optional
from typing import Text


from ..action import Action
from ..actions.opaque_function import OpaqueFunction
from ..frontend import Entity
from ..frontend import expose_action
from ..frontend import Parser
from ..launch_context import LaunchContext
from ..launch_description_entity import LaunchDescriptionEntity
from ..logging import get_logger
from ..some_substitutions_type import SomeSubstitutionsType
from ..substitution import Substitution
from ..utilities import perform_substitutions


@expose_action('for')
class ForLoop(Action):
    """
    Action that instantiates entities through a function N times based on a launch argument.

    A DeclareLaunchArgument must be created before this action to define the number of iterations
    in the for-loop, i.e., N iterations. For each loop iteration, the provided callback function is
    called with the index value, going from 0 to N (exclusive).

    Simple example:

    .. code-block:: python

        def for_i(i: int):
            return [
                LogInfo(msg=['i=', str(i)]),
            ]

        LaunchDescription([
            DeclareLaunchArgument('num', default_value='2'),
            ForLoop(LaunchConfiguration('num'), function=for_i),
        ])

    When using this action through a frontend, provide entities to be instantiated for each loop
    iteration as child entities. Use an $(index) substitution with the index name of the for-loop.

    Simple example:

    .. code-block:: xml

        <launch>
            <arg name="num" default="2" />
            <for len="$(var num)" name="i" >
                <log message="i=$(index i)" />
            </for>
        </launch>

    The above examples would ouput the following log messages by default:

    .. code-block:: text

        i=0
        i=1

    If the launch argument was set to 5 (num:=5), then it would output:

    .. code-block:: text

        i=0
        i=1
        i=2
        i=3
        i=4
    """

    def __init__(
        self,
        length: SomeSubstitutionsType,
        *,
        function: Callable[[int], Optional[List[LaunchDescriptionEntity]]],
        name: Optional[SomeSubstitutionsType] = None,
        **kwargs,
    ) -> None:
        """
        Create a ForLoop.

        :param length: the length of the for-loop
        :param function: a function that receives an index value and returns entities
        :param name: the for-loop name, used as the index name with the ForLoopIndex substitution
        """
        super().__init__(**kwargs)

        from ..utilities import normalize_to_list_of_substitutions  # import here to avoid loop
        self._length = normalize_to_list_of_substitutions(length)
        self._function = function
        self._name = normalize_to_list_of_substitutions(name) if name else []
        self._logger = get_logger(__name__)

    @property
    def length(self) -> List[Substitution]:
        return self._length

    @property
    def function(self) -> Callable[[int], Optional[List[LaunchDescriptionEntity]]]:
        return self._function

    @property
    def name(self) -> List[Substitution]:
        return self._name

    def describe(self) -> Text:
        return (
            type(self).__name__ +
            f"(length='{self._length}', name='{self._name}', function={self._function})"
        )

    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        """Return `ForLoop` action and kwargs for constructing it."""
        _, kwargs = super().parse(entity, parser)
        length = entity.get_attr('len')
        if length is not None:
            kwargs['length'] = parser.parse_substitution(length)
        name = entity.get_attr('name')
        kwargs['name'] = name
        parsed_children = [parser.parse_action(e) for e in entity.children]

        def for_i(i: int):
            return [
                # Push and pop locals to avoid having the index local leak
                OpaqueFunction(function=cls._push_locals),
                # Set a local equal to i so that it can be used as a unique value by the entities
                # through the ForLoopIndex substitution
                OpaqueFunction(function=cls._set_index_local, args=(name, i)),
                # Include a deep copy of parsed child entities
                *deepcopy(parsed_children),
                OpaqueFunction(function=cls._pop_locals),
            ]
        kwargs['function'] = for_i
        return cls, kwargs

    def execute(self, context: LaunchContext) -> Optional[List[LaunchDescriptionEntity]]:
        # Get the for-loop length and convert to int
        length = int(perform_substitutions(context, self._length))
        self._logger.debug(f'for-loop length={length}')

        entities = []
        for i in range(length):
            i_entities = self._function(i)
            if i_entities:
                entities.extend(i_entities)
        return entities

    @classmethod
    def _push_locals(
        cls,
        context: LaunchContext,
    ) -> Optional[List[LaunchDescriptionEntity]]:
        context._push_locals()
        return None

    @classmethod
    def _pop_locals(
        cls,
        context: LaunchContext,
    ) -> Optional[List[LaunchDescriptionEntity]]:
        context._pop_locals()
        return None

    @classmethod
    def _set_index_local(
        cls,
        context: LaunchContext,
        local_name: str,
        index: int,
    ) -> Optional[List[LaunchDescriptionEntity]]:
        context.extend_locals({local_name: str(index)})
        return None
