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

import contextlib

import launch
import launch.events


@contextlib.contextmanager
def launch_process(launch_service, process_action):
    """
    Launch a process.

    Start execution of a ``process_action`` using the given
    ``launch_service`` upon context entering and shut it down
    upon context exiting.
    """
    assert isinstance(process_action, launch.actions.ExecuteProcess)
    launch_service.emit_event(
        event=launch.events.IncludeLaunchDescription(
            launch_description=launch.LaunchDescription([process_action])
        )
    )
    try:
        yield process_action
    finally:
        launch_service.emit_event(
            event=launch.events.process.ShutdownProcess(
                process_matcher=launch.events.matches_action(process_action)
            )
        )
