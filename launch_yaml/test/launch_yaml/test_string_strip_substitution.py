# Copyright 2026 Open Source Robotics Foundation, Inc.
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

"""Test parsing a StringStripSubstitution in a YAML launch file."""

import io
import textwrap

from launch.actions import DeclareLaunchArgument
from launch.actions import SetLaunchConfiguration
from launch.frontend import Parser
from launch.launch_context import LaunchContext
from launch.substitutions import StringStripSubstitution


def test_nested():
    yaml_file = textwrap.dedent(
        """
        launch:
            - arg:
                name: robot_name
                default: "  rover  "
            - let:
                name: trimmed
                value: "$(string-strip $(var robot_name))"
        """
    )
    root_entity, parser = Parser.load(io.StringIO(yaml_file))
    launch_description = parser.parse_description(root_entity)

    assert len(launch_description.entities) == 2
    assert isinstance(launch_description.entities[0], DeclareLaunchArgument)
    assert isinstance(launch_description.entities[1], SetLaunchConfiguration)

    context = LaunchContext()
    launch_description.entities[0].visit(context)

    let = launch_description.entities[1]
    assert isinstance(let.value[0], StringStripSubstitution)
    assert let.value[0].perform(context) == 'rover'
