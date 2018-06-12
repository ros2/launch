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

"""Tests for normal usage of `launch_ros`."""

import functools
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))  # noqa
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'launch'))  # noqa

import launch.actions
import launch.event_handlers
# import launch.events
# import launch.substitutions
from launch import LaunchDescription
from launch import LaunchIntrospector
from launch import LaunchService

import launch_ros.actions
# import launch_ros.substitutions


def test_normal_case():
    """Test the normal use case for a ROS based launch description."""
    ld = LaunchDescription([
        launch.actions.ROSNodeContainer(name='container1', component_nodes=[
            launch_ros.actions.ROSNode(
                package='node_with_publisher',
                node_executable='node_with_publisher',
            ),
            launch_ros.actions.ROSNode(
                package='node_with_subscription',
                node_executable='node_with_subscription',
            ),
        ]),
    ])

    def on_output(event, *, file_name):
        for line in event.text.decode().splitlines():
            print('({}) {}'.format(event.process_name, line))

    ld.add_action(launch.actions.RegisterEventHandler(launch.event_handlers.OnProcessIO(
        on_stdout=functools.partial(on_output, file_name='stdout'),
        on_stderr=functools.partial(on_output, file_name='stderr'),
    )))

    print('Starting introspection of launch description...')
    print('')

    print(LaunchIntrospector().format_launch_description(ld))

    print('')
    print('Starting launch of launch description...')
    print('')

    # ls = LaunchService(debug=True)
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()


if __name__ == '__main__':
    test_normal_case()
