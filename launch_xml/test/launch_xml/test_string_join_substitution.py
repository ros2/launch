# Copyright 2025 Open Source Robotics Foundation, Inc.
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

"""Test parsing a StringJoinSubstitution in XML launch file."""

import io
import textwrap

from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.frontend import Parser
from launch.launch_context import LaunchContext
from launch.substitutions import StringJoinSubstitution


def test_nested():
    xml_file = textwrap.dedent(
        """
        <launch>
            <arg name="subdomain" default="wiki"/>
            <let name="url" value="$(string-join . https://$(var subdomain) ros org)"/>
        </launch>
        """
    )
    root_entity, parser = Parser.load(io.StringIO(xml_file))
    ld = parser.parse_description(root_entity)

    assert len(ld.entities) == 2
    assert isinstance(ld.entities[0], DeclareLaunchArgument)
    assert isinstance(ld.entities[1], SetLaunchConfiguration)

    lc = LaunchContext()
    ld.entities[0].visit(lc)

    let = ld.entities[1]
    assert isinstance(let.value[0], StringJoinSubstitution)
    assert let.value[0].perform(lc) == 'https://wiki.ros.org'


def test_delimiter():
    yaml_file = textwrap.dedent(
        """
        <launch>
            <arg name="delimiter" default="^_^"/>
            <let name="text" value="$(string-join '($(var delimiter))' a b c)"/>
        </launch>
        """
    )
    root_entity, parser = Parser.load(io.StringIO(yaml_file))
    ld = parser.parse_description(root_entity)

    assert len(ld.entities) == 2
    assert isinstance(ld.entities[0], DeclareLaunchArgument)
    assert isinstance(ld.entities[1], SetLaunchConfiguration)

    lc = LaunchContext()
    ld.entities[0].visit(lc)

    let = ld.entities[1]
    assert isinstance(let.value[0], StringJoinSubstitution)
    assert let.value[0].perform(lc) == 'a(^_^)b(^_^)c'
