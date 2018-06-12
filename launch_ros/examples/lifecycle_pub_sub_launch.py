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

"""Launch a talker and a listener."""

import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))  # noqa
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'launch'))  # noqa

import launch
import launch.actions
import launch.events

import launch_ros.actions
import launch_ros.events
import launch_ros.events.lifecycle
from launch_ros import get_default_ros_launch_description

import lifecycle_msgs.msg


def main(argv=sys.argv[1:]):
    """Main."""
    def _on_transition_event(context):
        print('got transition_event {}'.format(context.locals.event))

    ld = launch.LaunchDescription()

    # Launch the talker node.
    talker_node = launch_ros.actions.LifecycleNode(
        node_name='talker',
        package='lifecycle', node_executable='lifecycle_talker', output='screen')
    ld.add_action(talker_node)
    # When the talker node reaches the PRIMARY_STATE_ACTIVE state, start the listener.
    ld.add_action(launch.actions.RegisterEventHandler(launch_ros.event_handlers.OnStateTransition(
        target_lifecycle_node=talker_node, goal_state='active',
        entities=[
            launch.actions.LogInfo(
                msg="lifecycle node 'talker' reached the 'active' state, launching 'listener'."),
            launch_ros.actions.LifecycleNode(
                node_name='listener',
                package='lifecycle', node_executable='lifecycle_listener', output='screen'),
        ],
    )))

    # When the talker reaches 'configured', make it take the 'activate' transition.
    ld.add_action(launch.actions.RegisterEventHandler(launch_ros.event_handlers.OnStateTransition(
        target_lifecycle_node=talker_node, goal_state='inactive',
        entities=[
            launch.actions.LogInfo(
                msg="lifecycle node 'talker' reached the 'unconfigured' state, 'activating'."),
            launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                lifecycle_node_matcher=launch.events.process.matches_action(talker_node),
                transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
            )),
        ],
    )))

    # Make the talker node take the 'configure' transition.
    ld.add_action(launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
        lifecycle_node_matcher=launch.events.process.matches_action(talker_node),
        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
    )))

    print('Starting introspection of launch description...')
    print('')

    print(launch.LaunchIntrospector().format_launch_description(ld))

    print('')
    print('Starting launch of launch description...')
    print('')

    # ls = LaunchService(argv=argv, debug=True)
    ls = launch.LaunchService(argv=argv)
    ls.include_launch_description(get_default_ros_launch_description(prefix_output_with_name=False))
    ls.include_launch_description(ld)
    ls.run()


if __name__ == '__main__':
    main()
