# Copyright 2018 Open Source Robotics Foundation, Inc.
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

"""Module for the IncludeLaunchDescription action."""

import os
from typing import Any
from typing import Dict
from typing import Iterable
from typing import List
from typing import Optional
from typing import Sequence
from typing import Text
from typing import Tuple
from typing import Type
from typing import Union

import launch.logging

from .opaque_function import OpaqueFunction
from .set_launch_configuration import SetLaunchConfiguration
from ..action import Action
from ..frontend import Entity
from ..frontend import expose_action
from ..frontend import Parser
from ..launch_context import LaunchContext
from ..launch_description_entity import LaunchDescriptionEntity
from ..launch_description_source import LaunchDescriptionSource
from ..launch_description_sources import AnyLaunchDescriptionSource
from ..some_substitutions_type import SomeSubstitutionsType
from ..utilities import normalize_to_list_of_substitutions
from ..utilities import perform_substitutions


@expose_action('include')
class IncludeLaunchDescription(Action):
    """
    Action that includes a launch description source and yields its entities when visited.

    It is possible to pass arguments to the launch description, which it
    declared with the :py:class:`launch.actions.DeclareLaunchArgument` action.

    If any given arguments do not match the name of any declared launch
    arguments, then they will still be set as Launch Configurations using the
    :py:class:`launch.actions.SetLaunchConfiguration` action.
    This is done because it's not always possible to detect all instances of
    the declare launch argument class in the given launch description.

    On the other side, an error will sometimes be raised if the given launch
    description declares a launch argument and its value is not provided to
    this action.
    It will only produce this error, however, if the declared launch argument
    is unconditional (sometimes the action that declares the launch argument
    will only be visited in certain circumstances) and if it does not have a
    default value on which to fall back.

    Conditionally included launch arguments that do not have a default value
    will eventually raise an error if this best effort argument checking is
    unable to see an unsatisfied argument ahead of time.

    For example, to include ``my_pkg``'s ``other_launch.py`` and set launch arguments for it:

    .. code-block:: python

        def generate_launch_description():
            return ([
                SetLaunchConfiguration('arg1', 'value1'),
                IncludeLaunchDescription(
                    AnyLaunchDescriptionSource([
                        PathJoinSubstitution([
                            FindPackageShare('my_pkg'),
                            'launch',
                            'other_launch.py',
                        ]),
                    ]),
                    launch_arguments={
                        'other_arg1': LaunchConfiguration('arg1'),
                        'other_arg2': 'value2',
                    }.items(),
                ),
            ])

    .. code-block:: xml

        <launch>
            <let name="arg1" value="value1" />
            <include file="$(find-pkg-share my_pkg)/launch/other_launch.py">
                <let name="other_arg1" value="$(var arg1)" />
                <let name="other_arg2" value="value2" />
            </include>
        </launch>

    .. code-block:: yaml

        launch:
        - let:
            name: 'arg1'
            value: 'value1'
        - include:
            file: '$(find-pkg-share my_pkg)/launch/other_launch.py'
            let:
                - name: 'other_arg1'
                  value: '$(var arg1)'
                - name: 'other_arg2'
                  value: 'value2'

    .. note::

        While frontends currently support both ``let`` and ``arg`` for launch arguments, they are
        both converted into ``SetLaunchConfiguration`` actions (``let``). The same launch argument
        should not be defined using both ``let`` and ``arg``.
    """

    def __init__(
        self,
        launch_description_source: Union[LaunchDescriptionSource, SomeSubstitutionsType],
        *,
        launch_arguments: Optional[
            Iterable[Tuple[SomeSubstitutionsType, SomeSubstitutionsType]]
        ] = None,
        **kwargs: Any
    ) -> None:
        """Create an IncludeLaunchDescription action."""
        super().__init__(**kwargs)
        if not isinstance(launch_description_source, LaunchDescriptionSource):
            launch_description_source = AnyLaunchDescriptionSource(launch_description_source)
        self.__launch_description_source = launch_description_source
        self.__launch_arguments = () if launch_arguments is None else tuple(launch_arguments)
        self.__logger = launch.logging.get_logger(__name__)

    @classmethod
    def parse(cls, entity: Entity, parser: Parser
              ) -> Tuple[Type['IncludeLaunchDescription'], Dict[str, Any]]:
        """Return `IncludeLaunchDescription` action and kwargs for constructing it."""
        _, kwargs = super().parse(entity, parser)
        file_path = parser.parse_substitution(entity.get_attr('file'))
        kwargs['launch_description_source'] = file_path
        args = []
        args_arg = entity.get_attr('arg', data_type=List[Entity], optional=True)
        if args_arg is not None:
            args.extend(args_arg)
        args_let = entity.get_attr('let', data_type=List[Entity], optional=True)
        if args_let is not None:
            args.extend(args_let)
        if args:
            kwargs['launch_arguments'] = [
                (
                    parser.parse_substitution(e.get_attr('name')),
                    parser.parse_substitution(e.get_attr('value'))
                )
                for e in args
            ]
            for e in args:
                e.assert_entity_completely_parsed()
        return cls, kwargs

    @property
    def launch_description_source(self) -> LaunchDescriptionSource:
        """Getter for self.__launch_description_source."""
        return self.__launch_description_source

    @property
    def launch_arguments(self) -> Sequence[Tuple[SomeSubstitutionsType, SomeSubstitutionsType]]:
        """Getter for self.__launch_arguments."""
        return self.__launch_arguments

    def _get_launch_file(self) -> str:
        return os.path.abspath(self.__launch_description_source.location)

    def _get_launch_file_directory(self) -> str:
        launch_file_location = self._get_launch_file()
        if os.path.exists(launch_file_location):
            launch_file_location = os.path.dirname(launch_file_location)
        else:
            # If the location does not exist, then it's likely set to '<script>' or something
            # so just pass it along.
            launch_file_location = self.__launch_description_source.location
        return launch_file_location

    def get_sub_entities(self) -> List[LaunchDescriptionEntity]:
        """Get subentities."""
        ret = self.__launch_description_source.try_get_launch_description_without_context()
        return [ret] if ret is not None else []

    def _try_get_arguments_names_without_context(self) -> Optional[List[Text]]:
        try:
            context = LaunchContext()
            return [
                perform_substitutions(context, normalize_to_list_of_substitutions(arg_name))
                for arg_name, arg_value in self.__launch_arguments
            ]
        except Exception as exc:
            self.__logger.debug(
                'Failed to get launch arguments names for launch description '
                f"'{self.__launch_description_source.location}', "
                f'with exception: {str(exc)}'
            )
        return None

    def execute(self, context: LaunchContext) -> List[Union[SetLaunchConfiguration,
                                                            LaunchDescriptionEntity]]:
        """Execute the action."""
        launch_description = self.__launch_description_source.get_launch_description(context)
        self._set_launch_file_location_locals(context)

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
                    names = ild_action._try_get_arguments_names_without_context()
                    if names:
                        argument_names.extend(names)
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
            OpaqueFunction(function=self._restore_launch_file_location_locals),
        ]

    def _set_launch_file_location_locals(self, context: LaunchContext) -> None:
        context._push_locals()
        # Keep the previous launch file path/dir locals so that we can restore them after
        context_locals = context.get_locals_as_dict()
        self.__previous_launch_file_path = context_locals.get('current_launch_file_path', None)
        self.__previous_launch_file_dir = context_locals.get('current_launch_file_directory', None)
        context.extend_locals({
            'current_launch_file_path': self._get_launch_file(),
        })
        context.extend_locals({
            'current_launch_file_directory': self._get_launch_file_directory(),
        })

    def _restore_launch_file_location_locals(self, context: LaunchContext) -> None:
        # We want to keep the state of the context locals even after the include, since included
        # launch descriptions are meant to act as if they were included literally in the parent
        # launch description.
        # However, we want to restore the launch file path/dir locals to their previous state, and
        # we may have to just delete them if we're now going back to a launch script (i.e., not a
        # launch file). However, there is no easy way to delete context locals, so save current
        # locals, reset to the state before the include previous state and then re-apply locals,
        # potentially minus the launch file path/dir locals.
        context_locals = context.get_locals_as_dict()
        if self.__previous_launch_file_path is None:
            del context_locals['current_launch_file_path']
        else:
            context_locals['current_launch_file_path'] = self.__previous_launch_file_path
        if self.__previous_launch_file_dir is None:
            del context_locals['current_launch_file_directory']
        else:
            context_locals['current_launch_file_directory'] = self.__previous_launch_file_dir
        context._pop_locals()
        context.extend_locals(context_locals)

    def __repr__(self) -> Text:
        """Return a description of this IncludeLaunchDescription as a string."""
        return f'IncludeLaunchDescription({self.__launch_description_source.location})'
