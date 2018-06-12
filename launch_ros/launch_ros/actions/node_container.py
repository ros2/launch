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

"""Module for the NodeContainer action."""

import logging
from typing import Iterable
from typing import List
from typing import Optional

from launch.action import Action
from launch.actions import ExecuteProcess
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType

from .ros_node import ROSNode

_logger = logging.getLogger(name='launch_ros')


class NodeContainer(ExecuteProcess):
    """Action that starts a general purpose node component container and loads nods into it."""

    def __init__(
        self, *,
        name: SomeSubstitutionsType,
        component_nodes: Optional[Iterable[ROSNode]],
        **kwargs,
    ) -> None:
        """
        Construct an NodeContainer action.

        A container is created with the given name.

        The node actions given as components for the container to load are not
        necessarily visited as-is, but the information contained within them
        is used to load them into the container.

        This action will handle some events, in addition what
        `launch.actions.ExecuteProcess` handles:

        - `launch_ros.events.LoadComponentNodeInContainer`:

          - this event is emitted for each component node that should be loaded into a container
          - this action will see if the container name in the event matches this container, and if
            so it will load the component node into itself

        It will also emit some events in addition to the standard ones for a process:

        - `launch_ros.events.node.NodeExited`:

          - this event is emitted when a node exits
          - this happens whether or not it was a component unloading or a process exiting

        :param: package the package in which the node executable can be found
        :param: node_executable the name of the executable to find
        :param: arguments list of extra arguments for the node
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
