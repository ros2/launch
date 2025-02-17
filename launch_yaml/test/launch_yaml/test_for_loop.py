# Copyright 2024 Open Source Robotics Foundation, Inc.
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
from launch.actions import ForLoop
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.frontend import Parser
from launch.launch_context import LaunchContext
from launch.substitutions import ForLoopIndex
from launch.utilities import perform_substitutions


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
                    - log:
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
    # For each iteration:
    #   2 OpaqueFunction
    #   N user-defined entities
    #   1 OpaqueFunction
    # = 3 + N entitites/iteration
    assert len(actions) == 2 * (3 + 1)
    assert isinstance(actions[0], OpaqueFunction)
    assert isinstance(actions[1], OpaqueFunction)
    assert isinstance(actions[2], LogInfo)
    assert isinstance(actions[3], OpaqueFunction)
    assert isinstance(actions[4], OpaqueFunction)
    assert isinstance(actions[5], OpaqueFunction)
    assert isinstance(actions[6], LogInfo)
    assert isinstance(actions[7], OpaqueFunction)
    actions[0].visit(lc)
    actions[1].visit(lc)
    actions[2].visit(lc)
    assert isinstance(actions[2].msg[1], ForLoopIndex)
    assert perform_substitutions(lc, actions[2].msg[1].name) == 'i'
    assert perform_substitutions(lc, actions[2].msg) == 'index=0'
    actions[3].visit(lc)
    actions[4].visit(lc)
    actions[5].visit(lc)
    actions[6].visit(lc)
    assert isinstance(actions[6].msg[1], ForLoopIndex)
    assert perform_substitutions(lc, actions[6].msg[1].name) == 'i'
    assert perform_substitutions(lc, actions[6].msg) == 'index=1'
    actions[7].visit(lc)
