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

"""Test parsing an include action."""

import io
from pathlib import Path
import textwrap

from launch import LaunchDescription, LaunchDescriptionSource, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource

from parser_no_extensions import load_no_extensions


def test_include():
    """Parse node yaml example."""
    # Always use posix style paths in launch YAML files.
    path = (Path(__file__).parent / 'executable.yaml').as_posix()
    yaml_file = \
        """\
        launch:
        -   include:
                file: "{}"
        """.format(path)  # noqa: E501
    yaml_file = textwrap.dedent(yaml_file)
    root_entity, parser = load_no_extensions(io.StringIO(yaml_file))
    ld = parser.parse_description(root_entity)
    include = ld.entities[0]
    assert isinstance(include, IncludeLaunchDescription)
    assert isinstance(include.launch_description_source, AnyLaunchDescriptionSource)
    ls = LaunchService(debug=True)
    ls.include_launch_description(ld)
    assert 0 == ls.run()


def include_inner(inner_launch_file: str):
    # Always use posix style paths in launch YAML files.
    path = (Path(__file__).parent / inner_launch_file).as_posix()
    yaml_file = \
        """\
        launch:
        -   include:
                file: "{}"
        """.format(path)  # noqa: E501
    yaml_file = textwrap.dedent(yaml_file)
    root_entity, parser = load_no_extensions(io.StringIO(yaml_file))
    ld = parser.parse_description(root_entity)
    include = ld.entities[0]
    assert isinstance(include, IncludeLaunchDescription)
    assert isinstance(include.launch_description_source, AnyLaunchDescriptionSource)

    return ld


def test_include_inner_argument_default_no_argument():
    """Test inner launch file having an argument with default value (no commandline input)."""
    argument_name = 'inner_argument'
    ld = include_inner('inner_default.launch.yaml')

    ls = LaunchService(debug=True)
    # Pass the arguments as it is done in ros2launch
    ls.include_launch_description(LaunchDescription([
        IncludeLaunchDescription(LaunchDescriptionSource(ld), launch_arguments=[])
    ]))
    assert 0 == ls.run()
    assert len(ls.context.launch_configurations) == 1
    assert ls.context.launch_configurations == {argument_name: 'some default'}


def test_include_inner_argument_default_with_argument():
    """Test inner launch file having an argument with default value overwritten via commandline."""
    argument_name = 'inner_argument'
    argument_value = 'another_value'
    ld = include_inner('inner_default.launch.yaml')

    ls = LaunchService(debug=True, argv=[f'{argument_name}:="{argument_value}"'])

    # Pass the arguments as it is done in ros2launch
    ls.include_launch_description(LaunchDescription([
        IncludeLaunchDescription(
            LaunchDescriptionSource(ld),
            launch_arguments=[(argument_name, argument_value)]
        )
    ]))
    assert 0 == ls.run()
    assert len(ls.context.launch_configurations) == 1
    assert ls.context.launch_configurations == {argument_name: argument_value}


def test_include_inner_argument_no_argument():
    """Test inner launch file having a required argument value (no commandline input)."""
    ld = include_inner('inner.launch.yaml')

    ls = LaunchService(debug=True)
    # Pass the arguments as it is done in ros2launch
    ls.include_launch_description(LaunchDescription([
        IncludeLaunchDescription(LaunchDescriptionSource(ld), launch_arguments=[])
    ]))
    assert 1 == ls.run()
    assert len(ls.context.launch_configurations) == 0


def test_include_inner_argument_with_argument():
    """Test inner launch file having a required argument value overwritten via commandline."""
    argument_name = 'inner_argument'
    argument_value = 'another_value'
    ld = include_inner('inner.launch.yaml')

    ls = LaunchService(debug=True, argv=[f'{argument_name}:="{argument_value}"'])

    # Pass the arguments as it is done in ros2launch
    ls.include_launch_description(LaunchDescription([
        IncludeLaunchDescription(
            LaunchDescriptionSource(ld),
            launch_arguments=[(argument_name, argument_value)]
        )
    ]))
    assert 0 == ls.run()
    assert len(ls.context.launch_configurations) == 1
    assert ls.context.launch_configurations == {argument_name: argument_value}


if __name__ == '__main__':
    test_include()
    test_include_inner_argument_default_no_argument()
    test_include_inner_argument_default_with_argument()
    test_include_inner_argument_no_argument()
    test_include_inner_argument_with_argument()
