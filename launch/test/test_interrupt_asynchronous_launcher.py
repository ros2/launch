# Copyright 2016 Open Source Robotics Foundation, Inc.
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

import sys

from launch import LaunchDescriptor
from launch.launcher import AsynchronousLauncher
from launch.launcher import DefaultLauncher


def test_interrupt_asynchronous_launcher():
    desc = LaunchDescriptor()
    desc.add_process(
        cmd=[sys.executable, '-c', 'import time', 'time.sleep(30)'],
        name='test_interrupt_asynchronous_launcher__python_blocking'
    )

    launcher = DefaultLauncher()
    launcher.add_launch_descriptor(desc)
    async_launcher = AsynchronousLauncher(launcher)
    async_launcher.start()

    # wait up to 10 seconds to get to the point where at least all of the
    # asyncio-subprocess coroutines have been run (the processes are still
    # not guaranteed to be running yet)
    launcher.wait_on_processes_to_spawn(10)
    if not launcher.are_processes_spawned():
        # if the processes didn't launch after 10 seconds, fail the test
        assert False, 'launcher never reported processes launched'
    async_launcher.terminate()
    # now wait for the launcher to finish and error if if doesn't
    async_launcher.join(60)
    if async_launcher.is_alive():
        # if still running fail the test
        assert False, 'async launcher failed to shutdown'
