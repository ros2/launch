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

from pathlib import Path

from launch import LaunchDescription
from launch import LaunchService

from launch_frontend import Entity
from launch_frontend import parse_executable


def test_executable():
    """Parse node xml example."""
    root_entity = Entity.load(str(Path(__file__).parent / 'executable.xml'))
    executable = parse_executable(root_entity)
    cmd = [i[0].perform(None) for i in executable.cmd]
    assert(cmd ==
           ['ls', '-l', '-a', '-s'])
    assert(executable.cwd[0].perform(None) == '/')
    assert(executable.name[0].perform(None) == 'my_ls')
    assert(executable.shell is True)
    assert(executable.output == 'log')
    key, value = executable.additional_env[0]
    key = key[0].perform(None)
    value = value[0].perform(None)
    assert(key == 'var')
    assert(value == '1')
    assert(executable.prefix[0].perform(None) == 'time')
    ld = LaunchDescription()
    ld.add_action(executable)
    ls = LaunchService()
    ls.include_launch_description(ld)
    assert(0 == ls.run())


if __name__ == '__main__':
    test_executable()
