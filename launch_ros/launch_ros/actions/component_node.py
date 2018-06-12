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

"""Module for the ComponentNode action."""

import logging
from typing import List
from typing import Optional

from launch.action import Action
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitutions import LaunchConfiguration

from .node import Node
from ..substitutions import ExecutableInPackage

_logger = logging.getLogger(name='launch_ros.component_node')


class ComponentNode(Action):
    """Action that wraps a launch_ros.actions.Node and loads it into a node container."""

    def __init__(
        self, *,
        node: Node,
        node_container_name: SomeSubstitutionsType = LaunchConfiguration(name='node_container'),
    ) -> None:
        """
        Construct an ComponentNode action.

        This takes exactly one Node action, and does not visit it, but instead
        uses the information within it to instead load it into a node container
        by name.

        If no container name is given, it will attempt to be loaded from the
        launch configurations under the name 'node_container'.
        If not found there, a SubstitutionFailure error will occur.

        If the given node does not exist, it will fail the same as with the
        Node action used normally.
        If the given node exists, but is not a component-style node, a
        ValueError will occur.

        :param: node the Node action from which the component should be loaded
        :param: node_container_name the name of the node container to load into
        """
        cmd = [ExecutableInPackage(package=package, executable=node_executable)]
        cmd += [] if arguments is None else arguments
        super().__init__(cmd=cmd, **kwargs)
        self.__package = package
        self.__node_executable = node_executable
        self.__arguments = arguments

    def execute(self, context: LaunchContext) -> Optional[List[Action]]:
        """
        Execute the action.

        Delegated to :meth:`launch.actions.ExecuteProcess.execute`.
        """
        return super().execute(context)
