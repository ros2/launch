# Copyright 2019 Apex.AI, Inc.
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


import launch
import launch.actions

def test_multiple_launch_with_timers():
    # Regression test for https://github.com/ros2/launch/issues/183
    # Unfortunately, when things aren't working this test just hangs on the second call to
    # ls.run

    def generate_launch_description():
        return launch.LaunchDescription([
            launch.actions.TimerAction(
                period="1",
                actions=[
                    launch.actions.Shutdown(reason="Timer expired")
                ]
            )
        ])

    ls = launch.LaunchService()
    ls.include_launch_description(generate_launch_description())
    assert 0 == ls.run(shutdown_when_idle=False)  # Always works

    ls = launch.LaunchService()
    ls.include_launch_description(generate_launch_description())
    # Next line hangs forever before https://github.com/ros2/launch/issues/183 was fixed.
    assert 0 == ls.run(shutdown_when_idle=False)
