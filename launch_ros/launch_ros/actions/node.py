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

"""Module for the Node action."""

import logging
from typing import Dict  # noqa: F401
from typing import Iterable
from typing import List
from typing import Optional
from typing import Text  # noqa: F401
from typing import Tuple

from launch.action import Action
from launch.actions import ExecuteProcess
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitutions import LocalSubstitution
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions

from launch_ros.substitutions import ExecutableInPackage

from rclpy.validate_namespace import validate_namespace
from rclpy.validate_node_name import validate_node_name

_logger = logging.getLogger(name='launch_ros')


class Node(ExecuteProcess):
    """Action that executes a ROS node."""

    def __init__(
        self, *,
        package: SomeSubstitutionsType,
        node_executable: SomeSubstitutionsType,
        node_name: Optional[SomeSubstitutionsType] = None,
        node_namespace: Optional[SomeSubstitutionsType] = None,
        remappings: Optional[Iterable[Tuple[SomeSubstitutionsType, SomeSubstitutionsType]]] = None,
        arguments: Optional[Iterable[SomeSubstitutionsType]] = None,
        **kwargs
    ) -> None:
        """
        Construct an Node action.

        Many arguments are passed eventually to
        :class:`launch.actions.ExecuteProcess`, so see the documentation of
        that class for additional details.
        However, the `cmd` is not meant to be used, instead use the
        `node_executable` and `arguments` keyword arguments to this function.

        This action, once executed, delegates most work to the
        :class:`launch.actions.ExecuteProcess`, but it also converts some ROS
        specific arguments into generic command line arguments.

        The launch_ros.substitutions.ExecutableInPackage substitution is used
        to find the executable at runtime, so this Action also raise the
        exceptions that substituion can raise when the package or executable
        are not found.

        If the node_name is not given (or is None) then no name is passed to
        the node on creation and instead the default name specified within the
        code of the node is used instead.

        The node_namespace can either be absolute (i.e. starts with /) or
        relative.
        If absolute, then nothing else is considered and this is passed
        directly to the node to set the namespace.
        If relative, the namespace in the 'ros_namespace' LaunchConfiguration
        will be prepended to the given relative node namespace.
        If no node_namespace is given, then the default namespace `/` is
        assumed.

        :param: package the package in which the node executable can be found
        :param: node_executable the name of the executable to find
        :param: node_name the name of the node
        :param: node_namespace the ros namespace for this Node
        :param: remappings ordered list of 'to' and 'from' string pairs to be
            passed to the node as ROS remapping rules
        :param: arguments list of extra arguments for the node
        """
        cmd = [ExecutableInPackage(package=package, executable=node_executable)]
        cmd += [] if arguments is None else arguments
        # Reserve space for ros specific arguments.
        # The substitutions will get expanded when the action is executed.
        ros_args_index = 0
        if node_name is not None:
            cmd += [LocalSubstitution('ros_specific_arguments[{}]'.format(ros_args_index))]
            ros_args_index += 1
        if node_namespace is not None:
            cmd += [LocalSubstitution('ros_specific_arguments[{}]'.format(ros_args_index))]
            ros_args_index += 1
        if remappings is not None:
            for k, v in remappings:
                cmd += [LocalSubstitution('ros_specific_arguments[{}]'.format(ros_args_index))]
                ros_args_index += 1
        super().__init__(cmd=cmd, **kwargs)
        self.__package = package
        self.__node_executable = node_executable
        self.__node_name = node_name
        self.__node_namespace = node_namespace
        self.__remappings = [] if remappings is None else remappings
        self.__arguments = arguments

        self.__expanded_node_name = '<node_name_unspecified>'
        self.__expanded_node_namespace = '/'
        self.__final_node_name = None  # type: Optional[Text]
        self.__expanded_remappings = None  # type: Optional[Dict[Text, Text]]

        self.__substitutions_performed = False

    @property
    def node_name(self):
        """Getter for node_name."""
        if self.__final_node_name is None:
            raise RuntimeError("cannot access 'node_name' before executing action")
        return self.__final_node_name

    def _perform_substitutions(self, context: LaunchContext) -> None:
        try:
            if self.__substitutions_performed:
                # This function may have already been called by a subclass' `execute`, for example.
                return
            self.__substitutions_performed = True
            if self.__node_name is not None:
                self.__expanded_node_name = perform_substitutions(
                    context, normalize_to_list_of_substitutions(self.__node_name))
                validate_node_name(self.__node_name)
            self.__expanded_node_name.lstrip('/')
            if self.__node_namespace is not None:
                self.__expanded_node_namespace = perform_substitutions(
                    context, normalize_to_list_of_substitutions(self.__node_namespace))
            if not self.__expanded_node_namespace.startswith('/'):
                self.__expanded_node_namespace = '/' + self.__expanded_node_namespace
            validate_namespace(self.__expanded_node_namespace)
        except Exception:
            print("Error while expanding or validating node name or namespace for '{}':".format(
                'package={}, node_executable={}, name={}, namespace={}'.format(
                    self.__package,
                    self.__node_executable,
                    self.__node_name,
                    self.__node_namespace,
                )
            ))
            raise
        self.__final_node_name = ''
        if self.__expanded_node_namespace not in ['', '/']:
            self.__final_node_name += self.__expanded_node_namespace
        self.__final_node_name += '/' + self.__expanded_node_name
        # expand remappings too
        if self.__remappings is not None:
            self.__expanded_remappings = {}
            for k, v in self.__remappings:
                key = perform_substitutions(context, normalize_to_list_of_substitutions(k))
                value = perform_substitutions(context, normalize_to_list_of_substitutions(v))
                self.__expanded_remappings[key] = value

    def execute(self, context: LaunchContext) -> Optional[List[Action]]:
        """
        Execute the action.

        Delegated to :meth:`launch.actions.ExecuteProcess.execute`.
        """
        self._perform_substitutions(context)
        # Prepare the ros_specific_arguments list and add it to the context so that the
        # LocalSubstitution placeholders added to the the cmd can be expanded using the contents.
        ros_specific_arguments = []  # type: List[Text]
        if self.__node_name is not None:
            ros_specific_arguments.append('__node:={}'.format(self.__expanded_node_name))
        if self.__node_namespace is not None:
            ros_specific_arguments.append('__ns:={}'.format(self.__expanded_node_namespace))
        if self.__expanded_remappings is not None:
            for remapping_from, remapping_to in self.__remappings:
                ros_specific_arguments.append('{}:={}'.format(remapping_from, remapping_to))
        context.extend_locals({'ros_specific_arguments': ros_specific_arguments})
        return super().execute(context)
