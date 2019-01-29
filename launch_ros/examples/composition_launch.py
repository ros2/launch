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

"""Launch a talker and a listener in the same process."""

import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))  # noqa
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'launch'))  # noqa

from launch import LaunchDescription
from launch import LaunchIntrospector
from launch import LaunchService

from launch_ros import get_default_launch_description
import launch_ros.actions
import launch_ros.descriptions


def main(argv=sys.argv[1:]):
    """Main."""
    ld = LaunchDescription([
        launch_ros.actions.ComposableNodeContainer(
            package='rclcpp_components', container_executable='rclcpp_node_container', output='screen',
            # These are executed after the container process starts
            composable_node_descriptions=[
                launch_ros.descriptions.ComposableNode(
                    package_name='composition', node_plugin_name='talker',
                    name='my_talker'),
                launch_ros.descriptions.ComposableNode(
                    package_name='composition', node_plugin_name='listener'
                    name='my_listener')
            ]
        )
    ])

    print('Starting introspection of launch description...\n')

    print(LaunchIntrospector().format_launch_description(ld))

    print('\nStarting launch of launch description...\n')

    # ls = LaunchService(debug=True)
    ls = LaunchService()
    ls.include_launch_description(get_default_launch_description(prefix_output_with_name=False))
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == '__main__':
    main()
