# Copyright 2019 Open Source Robotics Foundation, Inc.
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

"""Module for the ComposableNodeContainer action."""

from typing import List
from typing import Optional

from launch.action import Action
from launch.launch_context import LaunchContext

from .node import Node

from ..descriptions import ComposableNode


class ComposableNodeContainer(Node):
    """Action that executes a container ROS node for composable ROS nodes."""

    def __init__(
        self,
        *,
        composable_node_descriptions: Optional[List[ComposableNode]] = None,
        **kwargs
    ) -> None:
        """
        Construct a ComposableNodeContainer action.

        Most arguments are forwarded to :class:`launch_ros.actions.Node`, so see
        the documentation of that class for further details.

        :param composable_node_descriptions: optional descriptions of composable nodes to be loaded
        """
        super(ComposableNodeContainer, self).__init__(**kwargs)
        self.__composable_node_descriptions = composable_node_descriptions

    def execute(self, context: LaunchContext) -> Optional[List[Action]]:
        """
        Execute the action.

        Most work is delegated to :meth:`launch_ros.actions.Node.execute`, except for the
        composable nodes load action if it applies.
        """
        load_actions = None  # type: Optional[List[Action]]
        if self.__composable_node_descriptions is not None:
            from .load_composable_node import LoadComposableNode
            load_actions = [LoadComposableNode(
                composable_node_descriptions=self.__composable_node_descriptions,
                target_container=self
            )]
        container_actions = super().execute(context)  # type: Optional[List[Action]]
        if container_actions is not None and load_actions is not None:
            return container_actions + load_actions
        if container_actions is not None:
            return container_actions
        if load_actions is not None:
            return load_actions
        return None
