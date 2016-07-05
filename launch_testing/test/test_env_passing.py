# Copyright 2015 Open Source Robotics Foundation, Inc.
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

import copy
import os
import sys

from launch import LaunchDescriptor
from launch.launcher import DefaultLauncher
from launch.exit_handler import primary_exit_handler


def test_env():
    ld = LaunchDescriptor()

    sub_env = copy.deepcopy(os.environ)
    sub_env['testenv1'] = 'testval1'
    os.environ['testenv2'] = 'testval2'
    ld.add_process(
        cmd=[
            sys.executable,
            os.path.join(
                os.path.abspath(
                    os.path.dirname(__file__)),
                'check_env.py')],
        name='test_env',
        env=sub_env,
        exit_handler=primary_exit_handler,
    )
    launcher = DefaultLauncher()
    launcher.add_launch_descriptor(ld)
    rc = launcher.launch()

    assert rc == 0, \
        "The launch file failed with exit code '" + str(rc) + "'. "


if __name__ == '__main__':
    test_env()
