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
import textwrap

from launch import LaunchService
from launch.actions import Shutdown
from launch.frontend import Parser


def test_executable():
    """Parse executable yaml example."""
    yaml_file = \
        """\
        launch:
        -   executable:
                cmd: ls -l -a -s
                cwd: '/'
                name: my_ls
                shell: true
                emulate_tty: true
                output: log
                sigkill_timeout: 4.0
                sigterm_timeout: 7.0
                'launch-prefix': $(env LAUNCH_PREFIX '')
                env:
                    -   name: var
                        value: '1'
        """
    yaml_file = textwrap.dedent(yaml_file)
    root_entity, parser = Parser.load(io.StringIO(yaml_file))
    ld = parser.parse_description(root_entity)
    executable = ld.entities[0]
    cmd = [i[0].perform(None) for i in executable.cmd]
    assert cmd == ['ls', '-l', '-a', '-s']
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


def test_executable_on_exit():
    yaml_file = \
        """\
        launch:
        -   executable:
                cmd: ls
                on_exit: shutdown
        """
    yaml_file = textwrap.dedent(yaml_file)
    root_entity, parser = Parser.load(io.StringIO(yaml_file))
    ld = parser.parse_description(root_entity)
    executable = ld.entities[0]
    sub_entities = executable.get_sub_entities()
    assert len(sub_entities) == 1
    assert isinstance(sub_entities[0], Shutdown)


if __name__ == '__main__':
    test_executable()
