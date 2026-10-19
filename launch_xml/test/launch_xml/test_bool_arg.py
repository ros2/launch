# Copyright 2026 Om Vaghasiya
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

"""Test parsing a ``DeclareBooleanLaunchArgument`` action."""

import io
import textwrap

from launch.actions import DeclareBooleanLaunchArgument

from parser_no_extensions import load_no_extensions

import pytest


def test_bool_arg():
    """Parse required and defaulted bool_arg actions."""
    xml_file = textwrap.dedent(
        """\
        <launch>
            <bool_arg name="required_arg"/>
            <bool_arg name="my_arg" default="true" description="something"/>
        </launch>
        """
    )
    root_entity, parser = load_no_extensions(io.StringIO(xml_file))
    launch_description = parser.parse_description(root_entity)
    required_argument, argument = launch_description.entities

    assert isinstance(required_argument, DeclareBooleanLaunchArgument)
    assert required_argument.name == 'required_arg'
    assert required_argument.default_value is None
    assert isinstance(argument, DeclareBooleanLaunchArgument)
    assert argument.name == 'my_arg'
    assert ''.join(
        substitution.perform(None) for substitution in argument.default_value
    ) == 'true'
    expected_choices = ['true', 'false', 'True', 'False', 'TRUE', 'FALSE']
    assert expected_choices == argument.choices
    assert argument.description == f'something. Valid choices are: {expected_choices}'


def test_bool_arg_choices_rejected():
    """Reject choices that override the boolean choices."""
    xml_file = textwrap.dedent(
        """\
        <launch>
            <bool_arg name="bool_arg_with_choices">
                <choice value="abc"/>
                <choice value="def"/>
            </bool_arg>
        </launch>
        """
    )
    root_entity, parser = load_no_extensions(io.StringIO(xml_file))
    with pytest.raises(ValueError, match='Cannot specify choices for bool_arg'):
        parser.parse_description(root_entity)
