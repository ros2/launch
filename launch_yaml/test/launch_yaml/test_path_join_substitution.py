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

"""Test parsing a `PathJoinSubstitution` substitution."""

import io
from pathlib import Path
import textwrap

import lark

from launch import LaunchService
from launch.actions import Log
from launch.substitutions import PathJoinSubstitution
from launch.utilities import perform_substitutions

from parser_no_extensions import load_no_extensions

import pytest


def test_path_join_substitution():
    yaml_file = \
        """\
        launch:
        - let:
            name: 'model'
            value: 'a'
        - let:
            name: 'id'
            value: '2'
        - let:
            name: 'ver'
            value: '1.0'
        - log:
            message: file=$(path-join "robot$(var id)" $(var model) "ur df" v_$(var ver).xacro)
        """
    yaml_file = textwrap.dedent(yaml_file)
    root_entity, parser = load_no_extensions(io.StringIO(yaml_file))
    launch_description = parser.parse_description(root_entity)

    ls = LaunchService(debug=True)
    ls.include_launch_description(launch_description)
    assert 0 == ls.run()

    log_info = launch_description.entities[3]
    assert isinstance(log_info, Log)
    assert isinstance(log_info.msg[1], PathJoinSubstitution)
    assert perform_substitutions(ls.context, log_info.msg) == \
        'file=' + str(Path('robot2', 'a', 'ur df', 'v_1.0.xacro'))


def test_path_join_substitution_empty():
    yaml_file = \
        """\
        launch:
        - log:
            message: file=$(path-join)
        """
    yaml_file = textwrap.dedent(yaml_file)
    root_entity, parser = load_no_extensions(io.StringIO(yaml_file))
    # lark hides the parsing error from the underlying Python class
    with pytest.raises(lark.exceptions.VisitError) as excinfo:
        parser.parse_description(root_entity)
    assert isinstance(excinfo.value.orig_exc, ValueError)
