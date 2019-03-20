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

"""Module containing the default LaunchDescription for ROS."""

import threading
from typing import Dict  # noqa: F401
from typing import Text  # noqa: F401
from typing import TextIO  # noqa: F401

import launch
import launch.actions
import launch.events

import rclpy
from rclpy.executors import SingleThreadedExecutor


class ROSSpecificLaunchStartup(launch.actions.OpaqueFunction):
    """Does ROS specific launch startup."""

    def __init__(self, rclpy_context=None):
        """Constructor."""
        super().__init__(function=self._function)
        self.__shutting_down = False
        self.__rclpy_context = rclpy_context

    def _shutdown(self, event: launch.Event, context: launch.LaunchContext):
        self.__shutting_down = True
        self.__rclpy_spin_thread.join()
        self.__launch_ros_node.destroy_node()

    def _run(self):
        executor = SingleThreadedExecutor(context=self.__rclpy_context)
        try:
            executor.add_node(self.__launch_ros_node)
            while not self.__shutting_down:
                # TODO(wjwwood): switch this to `spin()` when it considers
                #   asynchronously added subscriptions.
                #   see: https://github.com/ros2/rclpy/issues/188
                executor.spin_once(timeout_sec=1.0)
        except KeyboardInterrupt:
            pass
        finally:
            executor.remove_node(self.__launch_ros_node)

    def _function(self, context: launch.LaunchContext):
        try:
            if self.__rclpy_context is None:
                # Initialize the default global context
                rclpy.init(args=context.argv)
        except RuntimeError as exc:
            if 'rcl_init called while already initialized' in str(exc):
                pass
            raise
        self.__launch_ros_node = rclpy.create_node('launch_ros', context=self.__rclpy_context)
        context.extend_globals({
            'ros_startup_action': self,
            'launch_ros_node': self.__launch_ros_node
        })
        context.register_event_handler(launch.event_handlers.OnShutdown(
            on_shutdown=self._shutdown,
        ))
        self.__rclpy_spin_thread = threading.Thread(target=self._run)
        self.__rclpy_spin_thread.start()


def get_default_launch_description(*, rclpy_context=None):
    """
    Return a LaunchDescription to be included before user descriptions.

    :param: rclpy_context Provide a context other than the default rclpy context
        to pass down to rclpy.init.
        The context is expected to have already been initialized by the caller
        using rclpy.init().
    """
    default_ros_launch_description = launch.LaunchDescription([
        # ROS initialization (create node and other stuff).
        ROSSpecificLaunchStartup(rclpy_context=rclpy_context)
    ])
    return default_ros_launch_description
