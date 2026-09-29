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

"""Test parsing an include action."""

import io
from pathlib import Path
import textwrap

from launch import LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource

from parser_no_extensions import load_no_extensions


def test_include():
    """Parse include XML example."""
    path = (Path(__file__).parent / 'executable.xml').as_posix()
    xml_file = \
        """\
        <launch>
            <let name="main_baz" value="BAZ" />
            <include file="{}">
                <arg name="foo" value="FOO" />
                <arg name="baz" value="overwritten" />
                <let name="bar" value="BAR" />
                <let name="baz" value="$(var main_baz)" />
            </include>
        </launch>
        """.format(path)  # noqa: E501
    xml_file = textwrap.dedent(xml_file)
    root_entity, parser = load_no_extensions(io.StringIO(xml_file))
    ld = parser.parse_description(root_entity)
    include = ld.entities[1]
    assert isinstance(include, IncludeLaunchDescription)
    assert isinstance(include.launch_description_source, AnyLaunchDescriptionSource)
    ls = LaunchService(debug=True)
    ls.include_launch_description(ld)
    assert 0 == ls.run()
    assert ls.context.launch_configurations['foo'] == 'FOO'
    assert ls.context.launch_configurations['bar'] == 'BAR'
    assert ls.context.launch_configurations['baz'] == 'BAZ'


if __name__ == '__main__':
    test_include()
