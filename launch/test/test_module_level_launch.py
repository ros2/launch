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

import os
import sys
import time

from launch import LaunchDescriptor
from launch.launcher import AsynchronousLauncher
from launch.launcher import DefaultLauncher
from launch.loader import load_launch_file

async_launcher = None


def setup():
    global async_launcher
    default_launcher = DefaultLauncher()

    launch_file = os.path.join(os.path.abspath(os.path.dirname(__file__)), 'launch_counter.py')
    launch_descriptor = LaunchDescriptor()
    load_launch_file(launch_file, launch_descriptor, {})
    default_launcher.add_launch_descriptor(launch_descriptor)

    async_launcher = AsynchronousLauncher(default_launcher)
    async_launcher.start()


def teardown():
    global async_launcher
    if async_launcher:
        async_launcher.join()


def test_one():
    print('one', file=sys.stderr)
    time.sleep(1)


def test_two():
    print('two', file=sys.stderr)
    time.sleep(1)


def test_three():
    print('three', file=sys.stderr)
    time.sleep(1)


if __name__ == '__main__':
    setup()
    teardown()
