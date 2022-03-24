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

"""Test parsing a group action."""

import io
import textwrap

from launch.actions import PopEnvironment
from launch.actions import PopLaunchConfigurations
from launch.actions import PushEnvironment
from launch.actions import PushLaunchConfigurations
from launch.actions import SetLaunchConfiguration
from launch.frontend import Parser


def test_group():
    yaml_file = \
        """\
        launch:
            - group:
                scoped: False
                children:
                    - let:
                        name: 'var1'
                        value: 'asd'
                    - let:
                        name: 'var2'
                        value: 'asd'
        """  # noqa: E501
    yaml_file = textwrap.dedent(yaml_file)
    root_entity, parser = Parser.load(io.StringIO(yaml_file))
    ld = parser.parse_description(root_entity)
    group = ld.entities[0]
    actions = group.execute(None)
    assert 2 == len(actions)
    assert isinstance(actions[0], SetLaunchConfiguration)
    assert isinstance(actions[1], SetLaunchConfiguration)


def test_group_scoped():
    yaml_file = \
        """\
        launch:
            - group:
                scoped: True
                children:
                    - let:
                        name: 'var1'
                        value: 'asd'
                    - let:
                        name: 'var2'
                        value: 'asd'
        """  # noqa: E501
    yaml_file = textwrap.dedent(yaml_file)
    root_entity, parser = Parser.load(io.StringIO(yaml_file))
    ld = parser.parse_description(root_entity)
    group = ld.entities[0]
    actions = group.execute(None)
    assert 6 == len(actions)

    assert isinstance(actions[0], PushLaunchConfigurations)
    assert isinstance(actions[1], PushEnvironment)
    assert isinstance(actions[2], SetLaunchConfiguration)
    assert isinstance(actions[3], SetLaunchConfiguration)
    assert isinstance(actions[4], PopEnvironment)
    assert isinstance(actions[5], PopLaunchConfigurations)


if __name__ == '__main__':
    test_group()
