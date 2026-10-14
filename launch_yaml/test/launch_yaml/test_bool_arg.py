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


def test_bool_arg():
    """Parse a bool_arg action with a default and description."""
    yaml_file = textwrap.dedent(
        """\
        launch:
            - bool_arg:
                name: 'my_arg'
                default: 'true'
                description: 'something'
        """
    )
    root_entity, parser = load_no_extensions(io.StringIO(yaml_file))
    launch_description = parser.parse_description(root_entity)
    argument = launch_description.entities[0]

    assert isinstance(argument, DeclareBooleanLaunchArgument)
    assert argument.name == 'my_arg'
    assert ''.join(
        substitution.perform(None) for substitution in argument.default_value
    ) == 'true'
    assert argument.description == "something. Valid choices are: " \
        "['true', 'false', 'True', 'False', 'TRUE', 'FALSE']"
