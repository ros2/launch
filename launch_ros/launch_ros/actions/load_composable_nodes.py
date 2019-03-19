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

"""Module for the LoadComposableNodes action."""

import logging
from typing import List
from typing import Optional

import composition_interfaces.srv

from launch.action import Action
from launch.actions import RegisterEventHandler
from launch.event_handlers.on_process_start import OnProcessStart
from launch.events.process import ProcessStarted
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions

from .composable_node_container import ComposableNodeContainer

from ..descriptions import ComposableNode
from ..utilities import evaluate_parameters
from ..utilities import to_parameters_list

_logger = logging.getLogger(name='launch_ros')


class LoadComposableNodes(Action):
    """Action that loads composable ROS nodes into a running container."""

    def __init__(
        self,
        *,
        composable_nodes_description: List[ComposableNode],
        target_container: ComposableNodeContainer,
        in_parallel: Optional[SomeSubstitutionsType] = None,
        **kwargs,
    ) -> None:
        """
        Construct a LoadComposableNodes action.

        The container node is expected to provide a `~/_container/load_node` service for
        loading purposes.
        Loading can be performed sequentially for determinism or in parallel for speed.

        :param composable_nodes_description: descriptions of composable nodes to be loaded
        :param target_container: the container to load the nodes into
        :param in_parallel: whether to load nodes in parallel instead of sequentially.
            Defaults to False i.e. load sequentially
        """
        self.__composable_nodes_description = composable_nodes_description
        self.__target_container = target_container
        self.__in_parallel = None
        if in_parallel is not None:
            self.__in_parallel = normalize_to_list_of_substitutions(in_parallel)

    def _load_node(
        self,
        composable_node_description: ComposableNode,
        context: LaunchContext
    ) -> None:
        """
        Load node synchronously.

        :param composable_node_description: description of composable node to be loaded
        :param context: current launch context
        """
        while not self.__rclpy_load_node_client.wait_for_service(timeout_sec=1.0):
            if context.is_shutdown:
                _logger.warn(
                    "Abandoning wait for the '{}' service, due to shutdown.".format(
                        self.__rclpy_load_node_client.srv_name
                    )
                )
                return
        request = composition_interfaces.srv.LoadNode.Request()
        request.package_name = perform_substitutions(
            context, composable_node_description.package
        )
        request.plugin_name = perform_substitutions(
            context, composable_node_description.node_plugin
        )
        if composable_node_description.node_name is not None:
            request.node_name = perform_substitutions(
                context, composable_node_description.node_name
            )
        if composable_node_description.node_namespace is not None:
            request.node_namespace = perform_substitutions(
                context, composable_node_description.node_namespace
            )
        # request.log_level = perform_substitutions(context, node_description.log_level)
        if composable_node_description.remappings is not None:
            for from_, to in composable_node_description.remappings:
                request.remap_rules.append('{}:={}'.format(
                    perform_substitutions(context, list(from_)),
                    perform_substitutions(context, list(to)),
                ))
        if composable_node_description.parameters is not None:
            request.parameters = [
                param.to_parameter_msg() for param in to_parameters_list(
                    evaluate_parameters(context, composable_node_description.parameters)
                )
            ]
        if composable_node_description.extra_arguments is not None:
            request.extra_arguments = [
                param.to_parameter_msg() for param in to_parameters_list(
                    evaluate_parameters(context, composable_node_description.extra_arguments)
                )
            ]
        response = self.__rclpy_load_node_client.call(request)
        if not response.success:
            _logger.error("Failed to load node '{}' in container '{}': {}".format(
                response.full_node_name, self.__target_container.node_name, response.error_message
            ))

    def _load_in_sequence(
        self,
        composable_nodes_description: List[ComposableNode],
        context: LaunchContext
    ) -> None:
        """
        Load composable nodes sequentially.

        :param composable_nodes_description: descriptions of composable nodes to be loaded
        :param context: current launch context
        """
        next_composable_node_description = composable_nodes_description[0]
        composable_nodes_description = composable_nodes_description[1:]
        self._load_node(next_composable_node_description, context)
        if len(composable_nodes_description) > 0:
            context.add_completion_future(
                context.asyncio_loop.run_in_executor(
                    None, self._load_in_sequence, composable_nodes_description, context
                )
            )

    def _load_in_parallel(
        self,
        composable_nodes_description: List[ComposableNode],
        context: LaunchContext
    ) -> None:
        """
        Load composable nodes in parallel.

        :param composable_nodes_description: descriptions of composable nodes to be loaded
        :param context: current launch context
        """
        for composable_node_description in composable_nodes_description:
            context.add_completion_future(
                context.asyncio_loop.run_in_executor(
                    None, self._load_node, composable_node_description, context
                )
            )

    def _on_container_start(
        self,
        event: ProcessStarted,
        context: LaunchContext
    ) -> Optional[List[Action]]:
        """Load nodes on container process start."""
        in_parallel = False
        if self.__in_parallel is not None:
            in_parallel = bool(perform_substitutions(context, self.__in_parallel))
        load_method = self._load_in_parallel if in_parallel else self._load_in_sequence
        load_method(self.__composable_nodes_description, context)
        return None

    def execute(
        self,
        context: LaunchContext
    ) -> Optional[List[Action]]:
        """Execute the action."""
        # Create a client to load nodes in the target container.
        self.__rclpy_load_node_client = context.locals.launch_ros_node.create_client(
            composition_interfaces.srv.LoadNode, '{}/_container/load_node'.format(
                self.__target_container.node_name
            )
        )
        # Perform load action once the container has started.
        return [RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=self.__target_container,
                on_start=self._on_container_start,
            )
        )]
