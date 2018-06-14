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

import functools
import os
import threading
from typing import cast
from typing import Dict
from typing import Text
from typing import TextIO

import launch
import launch.actions
import launch.events

import rclpy

_process_log_files: Dict[Text, TextIO] = {}


def _on_process_started(context: launch.LaunchContext):
    typed_event = cast(launch.events.process.ProcessStarted, context.locals.event)
    if typed_event.execute_process_action.output == 'log':
        os.makedirs('/tmp/launch/logs', exist_ok=True)
        _process_log_files[typed_event.process_name] = open(
            '/tmp/launch/logs/{}.txt'.format(typed_event.process_name), 'w')


def _on_process_exited(context: launch.LaunchContext):
    typed_event = cast(launch.events.process.ProcessExited, context.locals.event)
    if typed_event.execute_process_action.output == 'log':
        print("closing log file '{}'".format(_process_log_files[typed_event.process_name].name))
        _process_log_files[typed_event.process_name].close()
        del _process_log_files[typed_event.process_name]


def _on_process_output(event: launch.Event, *, file_name: Text, prefix_output: bool):
    typed_event = cast(launch.events.process.ProcessIO, event)
    text = event.text.decode()
    if typed_event.execute_process_action.output == 'screen':
        if prefix_output:
            for line in text.splitlines():
                print('[{}] {}'.format(event.process_name, line))
        else:
            print(text, end='')
    elif typed_event.execute_process_action.output == 'log':
        if file_name == 'stderr':
            if prefix_output:
                for line in text.splitlines():
                    print('[{}:{}] {}'.format(event.process_name, file_name, line))
            else:
                print(text, end='')
        log_file = _process_log_files[typed_event.process_name]
        log_file.write(text)


class ROSSpecificLaunchStartup(launch.actions.OpaqueFunction):
    """Does ROS specific launch startup."""

    def __init__(self):
        """Constructor."""
        super().__init__(function=self._function)
        self.__shutting_down = False

    def _shutdown(self, event: launch.Event, context: launch.LaunchContext):
        rclpy.get_global_executor().shutdown()
        self.__rclpy_spin_thread.join()

    def _run(self):
        executor = rclpy.get_global_executor()
        try:
            executor.add_node(self.__launch_ros_node)
            while rclpy.ok():
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
            rclpy.init(args=context.argv)
        except RuntimeError as exc:
            if 'rcl_init called while already initialized' in str(exc):
                pass
            raise
        self.__launch_ros_node = rclpy.create_node('launch_ros')
        context.extend_globals({
            'ros_startup_action': self,
            'launch_ros_node': self.__launch_ros_node
        })
        context.register_event_handler(launch.event_handlers.OnShutdown(
            on_shutdown=self._shutdown,
        ))
        self.__rclpy_spin_thread = threading.Thread(target=self._run)
        self.__rclpy_spin_thread.start()


def get_default_launch_description(*, prefix_output_with_name=False):
    """
    Return a LaunchDescription to be included before user descriptions.

    :param: prefix_output_with_name if True, each line of output is prefixed
        with the name of the node as `[name] `, else it is printed unmodified
    """
    default_ros_launch_description = launch.LaunchDescription([
        # ROS initialization (create node and other stuff).
        ROSSpecificLaunchStartup(),
        # Handle process starts.
        launch.actions.RegisterEventHandler(launch.EventHandler(
            matcher=lambda event: isinstance(event, launch.events.process.ProcessStarted),
            entities=[launch.actions.OpaqueFunction(function=_on_process_started)],
        )),
        # Handle process exit.
        launch.actions.RegisterEventHandler(launch.EventHandler(
            matcher=lambda event: isinstance(event, launch.events.process.ProcessExited),
            entities=[launch.actions.OpaqueFunction(function=_on_process_exited)],
        )),
        # Add default handler for output from processes.
        launch.actions.RegisterEventHandler(launch.event_handlers.OnProcessIO(
            on_stdout=functools.partial(
                _on_process_output, file_name='stdout', prefix_output=prefix_output_with_name),
            on_stderr=functools.partial(
                _on_process_output, file_name='stderr', prefix_output=prefix_output_with_name),
        )),
    ])
    return default_ros_launch_description
