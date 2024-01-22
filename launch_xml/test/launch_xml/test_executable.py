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

"""Test parsing an executable action."""

import io
import os
import sys

from launch import LaunchService
from launch.actions import Shutdown
from launch.frontend import Parser

import pytest

test_executable_xml = f"""
<launch>
    <executable
        cmd="{sys.executable} --version"
        cwd="/" name="my_ls" shell="true" output="log"
        emulate_tty="true" sigkill_timeout="4.0" sigterm_timeout="7.0"
        launch-prefix="$(env LAUNCH_PREFIX '')"
    >
        <env name="var" value="1"/>
    </executable>
</launch>
"""


def test_executable():
    """Parse node xml example."""
    root_entity, parser = Parser.load(io.StringIO(test_executable_xml))
    ld = parser.parse_description(root_entity)
    executable = ld.entities[0]
    cmd = [i[0].perform(None) for i in executable.cmd]
    assert cmd == [sys.executable, '--version']
    assert executable.cwd[0].perform(None) == '/'
    assert executable.name[0].perform(None) == 'my_ls'
    assert executable.shell is True
    assert executable.emulate_tty is True
    assert executable.output[0].perform(None) == 'log'
    assert executable.sigkill_timeout[0].perform(None) == '4.0'
    assert executable.sigterm_timeout[0].perform(None) == '7.0'
    key, value = executable.additional_env[0]
    key = key[0].perform(None)
    value = value[0].perform(None)
    assert key == 'var'
    assert value == '1'
    ls = LaunchService()
    ls.include_launch_description(ld)
    assert 0 == ls.run()
    assert executable.return_code == 0


test_executable_wrong_subtag_xml = """
<launch>
    <executable cmd="some_command --that-does-not-matter">
        <env name="var" value="1"/>
        <whats_this/>
    </executable>
</launch>
"""


def test_executable_wrong_subtag():
    root_entity, parser = Parser.load(io.StringIO(test_executable_wrong_subtag_xml))
    with pytest.raises(ValueError) as excinfo:
        parser.parse_description(root_entity)
    assert '`executable`' in str(excinfo.value)
    assert 'whats_this' in str(excinfo.value)


split_arguments_example1 = f"""
<launch>
    <let name="args" value="--some-arg 'some string'" />
    <executable
        cmd="{sys.executable} {os.path.join(os.path.dirname(__file__), 'arg_echo.py')} $(var args)"
        log_cmd="True"
        split_arguments="True"
        output="screen"
    />
</launch>
"""


def test_executable_with_split_arguments():
    """Parse node xml example."""
    root_entity, parser = Parser.load(io.StringIO(split_arguments_example1))
    ld = parser.parse_description(root_entity)
    ls = LaunchService()
    ls.include_launch_description(ld)
    assert 0 == ls.run()


def test_executable_on_exit():
    xml_file = \
        """\
        <launch>
            <executable cmd="ls" on_exit="shutdown"/>
        </launch>
        """
    xml_file = textwrap.dedent(xml_file)
    root_entity, parser = Parser.load(io.StringIO(xml_file))
    ld = parser.parse_description(root_entity)
    executable = ld.entities[0]
    sub_entities = executable.get_sub_entities()
    assert len(sub_entities) == 1
    assert isinstance(sub_entities[0], Shutdown)


if __name__ == '__main__':
    test_executable()
