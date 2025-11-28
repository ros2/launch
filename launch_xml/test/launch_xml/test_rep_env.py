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

"""Test parsing a rep_env action."""

import io
import textwrap

from launch import LaunchContext
from launch.actions import ReplaceEnvironmentVariables
from launch.actions import SetLaunchConfiguration
from launch.frontend import Parser


def test_rep_env():
    xml_file = \
        """\
        <launch>
            <let name="bar" value="bar"/>
            <rep_env>
                <env name="foo" value="$(var bar)"/>
                <env name="spam" value="eggs"/>
                <env name="quux" value="quuux"/>
            </rep_env>
        </launch>
        """
    xml_file = textwrap.dedent(xml_file)
    root_entity, parser = Parser.load(io.StringIO(xml_file))
    ld = parser.parse_description(root_entity)

    assert len(ld.entities) == 2
    assert isinstance(ld.entities[0], SetLaunchConfiguration)
    assert isinstance(ld.entities[1], ReplaceEnvironmentVariables)

    lc = LaunchContext()
    ld.entities[0].visit(lc)
    ld.entities[1].execute(lc)

    assert 3 == len(lc.environment)
    assert 'foo' in lc.environment
    assert lc.environment['foo'] == 'bar'
    assert 'spam' in lc.environment
    assert lc.environment['spam'] == 'eggs'
    assert 'quux' in lc.environment
    assert lc.environment['quux'] == 'quuux'


def test_rep_env_no_envs():
    xml_file = \
        """\
        <launch>
            <rep_env/>
        </launch>
        """
    xml_file = textwrap.dedent(xml_file)
    root_entity, parser = Parser.load(io.StringIO(xml_file))
    ld = parser.parse_description(root_entity)

    assert len(ld.entities) == 1
    assert isinstance(ld.entities[0], ReplaceEnvironmentVariables)

    lc = LaunchContext()
    ld.entities[0].execute(lc)

    assert 0 == len(lc.environment)


if __name__ == '__main__':
    test_rep_env()
