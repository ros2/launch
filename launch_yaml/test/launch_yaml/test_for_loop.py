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

"""Test parsing a ForLoop action and a ForLoopIndex substitution."""

import io
import textwrap

from launch.actions import DeclareLaunchArgument
from launch.actions import ForEach
from launch.actions import ForLoop
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.frontend import Parser
from launch.launch_context import LaunchContext
from launch.substitutions import ForEachVar
from launch.substitutions import ForLoopIndex
from launch.utilities import perform_substitutions


def test_for_each():
    xml_file = textwrap.dedent(
        """
        launch:
            - arg:
                name: robots
                default: "{name: 'a', id: 1};{name: 'b', id: 2};{name: 'c', opt: '*'}"
            - for_each:
                values: $(var robots)
                children:
                    - log_info:
                        message: "'$(for-var name)' id=$(for-var id 0) ($(for-var opt 'none'))"
        """
    )
    root_entity, parser = Parser.load(io.StringIO(xml_file))
    ld = parser.parse_description(root_entity)

    assert len(ld.entities) == 2
    assert isinstance(ld.entities[0], DeclareLaunchArgument)
    assert isinstance(ld.entities[1], ForEach)

    lc = LaunchContext()
    ld.entities[0].visit(lc)
    actions = ld.entities[1].visit(lc)
    # For each iteration:
    #   2 OpaqueFunction
    #   N user-defined entities
    #   1 OpaqueFunction
    # = 3 + N entitites/iteration
    assert len(actions) == 3 * (3 + 1)
    assert isinstance(actions[0], OpaqueFunction)
    assert isinstance(actions[1], OpaqueFunction)
    assert isinstance(actions[2], LogInfo)
    assert isinstance(actions[3], OpaqueFunction)
    assert isinstance(actions[4], OpaqueFunction)
    assert isinstance(actions[5], OpaqueFunction)
    assert isinstance(actions[6], LogInfo)
    assert isinstance(actions[7], OpaqueFunction)
    assert isinstance(actions[8], OpaqueFunction)
    assert isinstance(actions[9], OpaqueFunction)
    assert isinstance(actions[10], LogInfo)
    assert isinstance(actions[11], OpaqueFunction)
    actions[0].visit(lc)
    actions[1].visit(lc)
    actions[2].visit(lc)
    assert isinstance(actions[2].msg[1], ForEachVar)
    assert perform_substitutions(lc, actions[2].msg[1].name) == 'name'
    assert perform_substitutions(lc, actions[2].msg[3].name) == 'id'
    assert perform_substitutions(lc, actions[2].msg[5].name) == 'opt'
    assert perform_substitutions(lc, actions[2].msg) == "'a' id=1 (none)"
    actions[3].visit(lc)
    actions[4].visit(lc)
    actions[5].visit(lc)
    actions[6].visit(lc)
    assert isinstance(actions[6].msg[1], ForEachVar)
    assert perform_substitutions(lc, actions[6].msg[1].name) == 'name'
    assert perform_substitutions(lc, actions[6].msg[3].name) == 'id'
    assert perform_substitutions(lc, actions[6].msg[5].name) == 'opt'
    assert perform_substitutions(lc, actions[6].msg) == "'b' id=2 (none)"
    actions[7].visit(lc)
    actions[8].visit(lc)
    actions[9].visit(lc)
    actions[10].visit(lc)
    assert isinstance(actions[10].msg[1], ForEachVar)
    assert perform_substitutions(lc, actions[10].msg[1].name) == 'name'
    assert perform_substitutions(lc, actions[10].msg[3].name) == 'id'
    assert perform_substitutions(lc, actions[10].msg[5].name) == 'opt'
    assert perform_substitutions(lc, actions[10].msg) == "'c' id=0 (*)"
    actions[11].visit(lc)


def test_for_loop():
    yaml_file = textwrap.dedent(
        """
        launch:
            - arg:
                name: num_i
                default: '2'
            - for:
                len: $(var num_i)
                name: i
                children:
                    - log_info:
                        message: index=$(index i)
        """
    )
    root_entity, parser = Parser.load(io.StringIO(yaml_file))
    ld = parser.parse_description(root_entity)

    assert len(ld.entities) == 2
    assert isinstance(ld.entities[0], DeclareLaunchArgument)
    assert isinstance(ld.entities[1], ForLoop)

    lc = LaunchContext()
    ld.entities[0].visit(lc)
    actions = ld.entities[1].visit(lc)
    assert len(actions) == 1
    assert isinstance(actions[0], ForEach)
    actions_for_each = actions[0].visit(lc)
    # For each iteration:
    #   2 OpaqueFunction
    #   N user-defined entities
    #   1 OpaqueFunction
    # = 3 + N entitites/iteration
    assert len(actions_for_each) == 2 * (3 + 1)
    assert isinstance(actions_for_each[0], OpaqueFunction)
    assert isinstance(actions_for_each[1], OpaqueFunction)
    assert isinstance(actions_for_each[2], LogInfo)
    assert isinstance(actions_for_each[3], OpaqueFunction)
    assert isinstance(actions_for_each[4], OpaqueFunction)
    assert isinstance(actions_for_each[5], OpaqueFunction)
    assert isinstance(actions_for_each[6], LogInfo)
    assert isinstance(actions_for_each[7], OpaqueFunction)
    actions_for_each[0].visit(lc)
    actions_for_each[1].visit(lc)
    actions_for_each[2].visit(lc)
    assert isinstance(actions_for_each[2].msg[1], ForLoopIndex)
    assert perform_substitutions(lc, actions_for_each[2].msg[1].name) == 'i'
    assert perform_substitutions(lc, actions_for_each[2].msg) == 'index=0'
    actions_for_each[3].visit(lc)
    actions_for_each[4].visit(lc)
    actions_for_each[5].visit(lc)
    actions_for_each[6].visit(lc)
    assert isinstance(actions_for_each[6].msg[1], ForLoopIndex)
    assert perform_substitutions(lc, actions_for_each[6].msg[1].name) == 'i'
    assert perform_substitutions(lc, actions_for_each[6].msg) == 'index=1'
    actions_for_each[7].visit(lc)
