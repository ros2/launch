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

"""Module for the ForLoop action."""

from copy import deepcopy
from typing import Any
from typing import Callable
from typing import Dict
from typing import List
from typing import Mapping
from typing import Optional
from typing import Protocol
from typing import Text
from typing import Tuple
from typing import Type


# yaml has type annotations in typeshed, but those cannot be installed via rosdep
# since there is no definition for types-PyYAML
import yaml  # type: ignore

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
from ..substitutions import ForEachVar
from ..utilities import perform_substitutions


@expose_action('for_each')
class ForEach(Action):
    """
    Action that iterates through sets of input values and uses them to instantiate entities.

    Sets of input values are provided as semicolon-separated string YAML structures, which could be
    a substitution, such as a :class:`launch.substitutions.LaunchConfiguration`. Each iteration
    gets a set of input values it can use to instantiate entities. For example, the values can be
    used to create the same node but under different namespaces. The number of iterations is
    defined by the number of semicolon-separated sets of values. An empty string results in no
    iterations, while an empty YAML structure (e.g., `'{}'`) results in an iteration with no input
    values.

    When using this action directly through Python, for each iteration, the provided callback
    function is called with one set of values, and should return a list of entities. The names of
    the callback function parameters must match the keys in the YAML structure, and the expected
    types of the parameters correspond to the types of the values in the YAML structure (YAML type
    rules apply). The order of the callback function parameters does not matter. The callback
    function could also use `**kwargs`. Finally, default values can be defined through default
    values of callback function parameters, in which case they may be omitted from a set of values
    in the YAML string.

    Simple example:

    .. code-block:: python

        def for_each(id: int, name: str):
            return [
                LogInfo(msg=f"robot '{name}' id={id}"),
            ]

        def generate_launch_description():
            return LaunchDescription([
                DeclareLaunchArgument(
                    'robots', default_value="{name: 'robotA', id: 1};{name: 'robotB', id: 2}"),
                ForEach(LaunchConfiguration('robots'), function=for_each),
            ])

    When using this action through a frontend, provide entities to be instantiated for each loop
    iteration as child entities. Use a `$(for-var)` substitution (:class:`ForEachVar`) with the
    name of the for-each variable, e.g., `$(for-var name)`. A default value can be provided for the
    variable if it is not available for a given iteration, e.g., `$(for-var name default)`.

    Simple examples:

    .. code-block:: xml

        <launch>
            <arg name="robots" default="{name: 'robotA', id: 1};{name: 'robotB', id: 2}" />
            <for_each values="$(var robots)" >
                <log_info message="'$(for-var name)' id=$(for-var id)" />
            </for_each>
        </launch>

    .. code-block:: yaml

        launch:
            - arg:
                name: robots
                default: "{name: 'robotA', id: 1};{name: 'robotB', id: 2}"
            - for_each:
                iter: $(var robots)
                children:
                    - log_info:
                        message: "'$(for-var name)' id=$(for-var id)"

    The above examples would ouput the following log messages by default:

    .. code-block:: text

        'robotA' id=1
        'robotB' id=2

    If the 'robots' launch argument was set to a different value:

    .. code-block:: console

        robots:="{name: 'robotC', id: 3};{name: 'robotD', id: 4};{name: 'robotE', id: 5}"

    Then it would output:

    .. code-block:: text

        'robotC' id=3
        'robotD' id=4
        'robotE' id=5
    """

    SEPARATOR = ';'

    def __init__(
        self,
        input_values: SomeSubstitutionsType,
        *,
        function: Callable[..., Optional[List[LaunchDescriptionEntity]]],
        **kwargs,
    ) -> None:
        """
        Create a ForEach.

        :param input_values: the sets of inputs values to iterate over, provided as a
            semicolon-separated list of string YAML structures (e.g., flow style YAML strings
            separated by semicolons)
        :param function: a function that receives values from each YAML structure and returns
            entities
        """
        super().__init__(**kwargs)

        from ..utilities import normalize_to_list_of_substitutions  # import here to avoid loop
        self._input_values = normalize_to_list_of_substitutions(input_values)
        self._function = function
        self._logger = get_logger(__name__)

    @property
    def input_values(self) -> List[Substitution]:
        return self._input_values

    @property
    def function(self) -> Callable[[int], Optional[List[LaunchDescriptionEntity]]]:
        return self._function

    def describe(self) -> Text:
        return (
            self.__class__.__name__ +
            f"(input_values='{self._input_values}', function={self._function})"
        )

    @classmethod
    def parse(cls, entity: Entity, parser: Parser
              ) -> Tuple[Type['ForEach'], Dict[str, Any]]:
        """Return `ForEach` action and kwargs for constructing it."""
        _, kwargs = super().parse(entity, parser)
        input_values = entity.get_attr('values')
        if input_values is not None:
            kwargs['input_values'] = parser.parse_substitution(input_values)
        parsed_children = [parser.parse_action(e) for e in entity.children]

        def for_each(**iteration_vars) -> List[LaunchDescriptionEntity]:
            return cls._get_iteration_entities(parsed_children, iteration_vars)
        kwargs['function'] = for_each
        return cls, kwargs

    def execute(self, context: LaunchContext) -> List[LaunchDescriptionEntity]:
        # Get the for-each input values
        input_values = perform_substitutions(context, self._input_values)
        self._logger.debug(f'input_values={input_values}')
        # Split into list of dicts
        input_values_list = list(filter(None, input_values.strip().split(self.SEPARATOR)))
        iteration_dicts = [yaml.safe_load(i.strip()) for i in input_values_list]
        if not iteration_dicts:
            self._logger.warning('no input values: will not iterate')

        entities = []
        for iteration_dict in iteration_dicts:
            self._logger.debug(f'iteration: {iteration_dict}')
            # Do still call the function if the YAML structure is empty
            if iteration_dict is not None:
                i_entities = self._function(**iteration_dict)
                if i_entities:
                    entities.extend(i_entities)
        return entities

    @classmethod
    def _get_iteration_entities(
        cls,
        children: List[Action],
        iteration_vars: Mapping[str, Any],
    ) -> List[LaunchDescriptionEntity]:
        return [
            # Push and pop locals to avoid having the local variables leak
            OpaqueFunction(function=cls._push_locals),
            # Set local variables so that they can be used as iteration-specific values by the
            # child entities through substitutions
            OpaqueFunction(function=cls._set_args_local, args=(iteration_vars,)),
            # Include a deep copy of child entities
            *deepcopy(children),
            OpaqueFunction(function=ForEach._pop_locals),
        ]

    @classmethod
    def _push_locals(
        cls,
        context: LaunchContext,
    ) -> None:
        context._push_locals()
        return None

    @classmethod
    def _pop_locals(
        cls,
        context: LaunchContext,
    ) -> None:
        context._pop_locals()
        return None

    @classmethod
    def _set_args_local(
        cls,
        context: LaunchContext,
        args: Mapping[str, Any],
    ) -> Optional[List[LaunchDescriptionEntity]]:
        context.extend_locals(
            {ForEachVar.get_local_arg_name(name): str(value) for name, value in args.items()})
        return None


@expose_action('for')
class ForLoop(Action):
    """
    Action that instantiates entities through a function N times, e.g., based on a launch argument.

    The number of iterations of the for-loop is defined by the provided length, which could be a
    substitution, such as a :class:`launch.substitutions.LaunchConfiguration`.

    When using this action directly through Python, for each loop iteration, the provided callback
    function is called with the index value, going from 0 to N (exclusive), and should return a
    list of entities. The callback function must have one parameter: `i` of type `int`.

    Simple example:

    .. code-block:: python

        def for_i(i: int):
            return [
                LogInfo(msg=['i=', str(i)]),
            ]

        def generate_launch_description():
            return LaunchDescription([
                DeclareLaunchArgument('num', default_value='2'),
                ForLoop(LaunchConfiguration('num'), function=for_i),
            ])

    When using this action through a frontend, provide entities to be instantiated for each loop
    iteration as child entities. Use an `$(index)` substitution
    (:class:`launch.substitutions.ForLoopIndex`) with the index name of the for-loop.

    Simple examples:

    .. code-block:: xml

        <launch>
            <arg name="num" default="2" />
            <for len="$(var num)" name="i" >
                <log_info message="i=$(index i)" />
            </for>
        </launch>

    .. code-block:: yaml

        launch:
            - arg:
                name: num
                default: '2'
            - for:
                len: $(var num)
                name: i
                children:
                    - log_info:
                        message: i=$(index i)

    The above examples would ouput the following log messages by default:

    .. code-block:: text

        i=0
        i=1

    If the 'num' launch argument was set to 5 (num:=5), then it would output:

    .. code-block:: text

        i=0
        i=1
        i=2
        i=3
        i=4
    """

    class _CallbackFunction(Protocol):

        def __call__(self, i: int) -> Optional[List[LaunchDescriptionEntity]]: ...

    def __init__(
        self,
        length: SomeSubstitutionsType,
        *,
        function: _CallbackFunction,
        name: Optional[str] = None,
        **kwargs,
    ) -> None:
        """
        Create a ForLoop.

        :param length: the length of the for-loop; must be convertible to `int` through `int()`,
            otherwise a `ValueError` will be raised during execution
        :param function: a function that receives an integer loop index value (`i`) and returns
            entities
        :param name: the for-loop name, used as the index name with the ForLoopIndex substitution
        """
        super().__init__(**kwargs)

        from ..utilities import normalize_to_list_of_substitutions  # import here to avoid loop
        self._length = normalize_to_list_of_substitutions(length)
        self._function = function
        self._name = name
        self._logger = get_logger(__name__)

    @property
    def length(self) -> List[Substitution]:
        return self._length

    @property
    def function(self) -> Callable[[int], Optional[List[LaunchDescriptionEntity]]]:
        return self._function

    @property
    def name(self) -> Optional[str]:
        return self._name

    def describe(self) -> Text:
        return (
            self.__class__.__name__ +
            f"(length='{self._length}', name='{self._name}', function={self._function})"
        )

    @classmethod
    def parse(cls, entity: Entity, parser: Parser
              ) -> Tuple[Type['ForLoop'], Dict[str, Any]]:
        """Return `ForLoop` action and kwargs for constructing it."""
        _, kwargs = super().parse(entity, parser)
        length = entity.get_attr('len')
        if length is not None:
            kwargs['length'] = parser.parse_substitution(length)
        name = entity.get_attr('name')
        kwargs['name'] = name
        parsed_children = [parser.parse_action(e) for e in entity.children]

        def for_i(i: int) -> List[LaunchDescriptionEntity]:
            return ForEach._get_iteration_entities(parsed_children, {name: i})
        kwargs['function'] = for_i
        return cls, kwargs

    def execute(self, context: LaunchContext) -> List[ForEach]:
        # Get the for-loop length and convert to int
        length = int(perform_substitutions(context, self._length))
        self._logger.debug(f'for-loop length={length}')
        # Create list of YAML (dict) strings
        input_values = ForEach.SEPARATOR.join(
            yaml.dump({'i': i}, default_flow_style=True).strip()
            for i in range(length)
        )
        self._logger.debug(f'input_values={input_values}')
        return [ForEach(input_values, function=self._function)]
