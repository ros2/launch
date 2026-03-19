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


def test_include_scoped_true():
    """Parse include with scoped="true" — child configs do not leak to parent."""
    path = (Path(__file__).parent / 'executable.xml').as_posix()
    xml_file = \
        """\
        <launch>
            <let name="bar" value="BAR" />
            <include file="{}" scoped="true">
                <let name="foo" value="FOO" />
            </include>
        </launch>
        """.format(path)  # noqa: E501
    xml_file = textwrap.dedent(xml_file)
    root_entity, parser = load_no_extensions(io.StringIO(xml_file))
    ld = parser.parse_description(root_entity)
    include = ld.entities[1]
    assert isinstance(include, IncludeLaunchDescription)
    ls = LaunchService(debug=True)
    ls.include_launch_description(ld)
    assert 0 == ls.run()
    # bar persists, but foo from scoped include does not leak
    assert ls.context.launch_configurations['bar'] == 'BAR'
    assert 'foo' not in ls.context.launch_configurations


def test_include_scoped_false():
    """Parse include with scoped="false" — child configs leak to parent (default behavior)."""
    path = (Path(__file__).parent / 'executable.xml').as_posix()
    xml_file = \
        """\
        <launch>
            <let name="bar" value="BAR" />
            <include file="{}" scoped="false">
                <let name="foo" value="FOO" />
            </include>
        </launch>
        """.format(path)  # noqa: E501
    xml_file = textwrap.dedent(xml_file)
    root_entity, parser = load_no_extensions(io.StringIO(xml_file))
    ld = parser.parse_description(root_entity)
    include = ld.entities[1]
    assert isinstance(include, IncludeLaunchDescription)
    ls = LaunchService(debug=True)
    ls.include_launch_description(ld)
    assert 0 == ls.run()
    # Both bar and foo are visible
    assert ls.context.launch_configurations['bar'] == 'BAR'
    assert ls.context.launch_configurations['foo'] == 'FOO'


def test_include_default_is_unscoped():
    """Parse include without scoped attribute — defaults to unscoped (backward compatible)."""
    path = (Path(__file__).parent / 'executable.xml').as_posix()
    xml_file = \
        """\
        <launch>
            <include file="{}">
                <let name="foo" value="FOO" />
            </include>
        </launch>
        """.format(path)  # noqa: E501
    xml_file = textwrap.dedent(xml_file)
    root_entity, parser = load_no_extensions(io.StringIO(xml_file))
    ld = parser.parse_description(root_entity)
    include = ld.entities[0]
    assert isinstance(include, IncludeLaunchDescription)
    ls = LaunchService(debug=True)
    ls.include_launch_description(ld)
    assert 0 == ls.run()
    # foo leaks, same as before
    assert ls.context.launch_configurations['foo'] == 'FOO'


if __name__ == '__main__':
    test_include()
