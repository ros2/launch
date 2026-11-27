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

"""Test parsing a `DeclareLaunchArgument` action."""

import io
import textwrap

from parser_no_extensions import load_no_extensions

import pytest


def test_arg():
    """Parse a required arg and a defaulted arg."""
    yaml_file = textwrap.dedent(
        """\
        launch:
            - arg:
                name: 'required_arg'
            - arg:
                name: 'my_arg'
                default: 'asd'
                description: 'something'
        """
    )
    root_entity, parser = load_no_extensions(io.StringIO(yaml_file))
    launch_description = parser.parse_description(root_entity)
    required_argument, argument = launch_description.entities

    assert required_argument.name == 'required_arg'
    assert required_argument.default_value is None
    assert argument.name == 'my_arg'
    assert 'asd' == ''.join(
        substitution.perform(None) for substitution in argument.default_value
    )
    assert 'something' == argument.description
    assert argument.choices is None


def test_arg_with_choices_as_dicts():
    """Parse choices given in the list-of-dictionaries `choice` form."""
    yaml_file = textwrap.dedent(
        """\
        launch:
            - arg:
                name: 'my_arg'
                default: 'asd'
                description: 'something'
                choice:
                    - value: 'asd'
                    - value: 'bsd'
        """
    )
    root_entity, parser = load_no_extensions(io.StringIO(yaml_file))
    launch_description = parser.parse_description(root_entity)
    arg = launch_description.entities[0]

    assert 'my_arg' == arg.name
    assert ['asd', 'bsd'] == arg.choices
    assert "something. Valid choices are: ['asd', 'bsd']" == arg.description


def test_arg_with_choices_as_plain_list():
    """Parse choices given as a plain list of values under `choice`."""
    yaml_file = textwrap.dedent(
        """\
        launch:
            - arg:
                name: 'my_arg'
                default: 'asd'
                description: 'something'
                choice: [asd, bsd]
        """
    )
    root_entity, parser = load_no_extensions(io.StringIO(yaml_file))
    launch_description = parser.parse_description(root_entity)
    arg = launch_description.entities[0]

    assert 'my_arg' == arg.name
    assert ['asd', 'bsd'] == arg.choices
    assert "something. Valid choices are: ['asd', 'bsd']" == arg.description


def test_arg_with_choices_key_as_plain_list():
    """Parse choices given under the `choices` key as a plain list of values."""
    yaml_file = textwrap.dedent(
        """\
        launch:
            - arg:
                name: 'my_arg'
                default: 'asd'
                description: 'something'
                choices: [asd, bsd]
        """
    )
    root_entity, parser = load_no_extensions(io.StringIO(yaml_file))
    launch_description = parser.parse_description(root_entity)
    arg = launch_description.entities[0]

    assert 'my_arg' == arg.name
    assert ['asd', 'bsd'] == arg.choices
    assert "something. Valid choices are: ['asd', 'bsd']" == arg.description


def test_arg_with_choices_key_as_dicts():
    """Parse choices given under the `choices` key as dictionaries."""
    yaml_file = textwrap.dedent(
        """\
        launch:
            - arg:
                name: 'my_arg'
                default: 'asd'
                description: 'something'
                choices:
                    - value: 'asd'
                    - value: 'bsd'
        """
    )
    root_entity, parser = load_no_extensions(io.StringIO(yaml_file))
    launch_description = parser.parse_description(root_entity)
    arg = launch_description.entities[0]

    assert 'my_arg' == arg.name
    assert ['asd', 'bsd'] == arg.choices


def test_arg_with_both_choice_and_choices_rejected():
    """Reject an arg that specifies both `choice` and `choices`."""
    yaml_file = textwrap.dedent(
        """\
        launch:
            - arg:
                name: 'my_arg'
                default: 'asd'
                choice: [asd, bsd]
                choices: [asd, bsd]
        """
    )
    root_entity, parser = load_no_extensions(io.StringIO(yaml_file))
    with pytest.raises(ValueError, match="only one of 'choice' and 'choices'"):
        parser.parse_description(root_entity)
