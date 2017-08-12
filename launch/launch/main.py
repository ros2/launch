# Copyright 2017 Open Source Robotics Foundation, Inc.
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

import argparse
import os
import sys

from launch import LaunchDescriptor
from launch.launcher import DefaultLauncher
from launch.loader import load_launch_file


def file_exists(filename):
    return os.path.exists(filename) and os.path.isfile(filename)


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(
        description='Launch the processes specified in a launch file.')
    parser.add_argument(
        'launch_file',
        type=str,
        nargs='+',
        help='The launch file.')
    parser.add_argument(
        'arg',
        type=str,
        nargs='*',
        help='An argument to the launch file (e.g., arg_name:=value).')
    args = parser.parse_args(argv)

    # Get the list of launch files passed on the command line
    # NOTE: since both the launch files and launch arguments are consecutive
    # lists of arguments, argparse will put all arguments into the launch_file
    # list, and the arg list will be empty. This is because the '+' list is
    # greedy and consumes the remaining arguments. Therefore the arguments
    # need to be validated afterward
    launch_files = []
    for arg in args.launch_file:
        # Store consecutive existing files and stop once a
        # non-existing file is found
        if file_exists(arg):
            launch_files.append(arg)
        else:
            break

    # Ensure that at least one existing launch file was given
    if len(launch_files) == 0:
        print('error: you must pass at least one valid launch file', file=sys.stderr)
        parser.print_help()
        sys.exit(2)

    launcher = DefaultLauncher()
    for launch_file in launch_files:
        launch_descriptor = LaunchDescriptor()
        load_launch_file(launch_file, launch_descriptor, argv)
        launcher.add_launch_descriptor(launch_descriptor)
    rc = launcher.launch()
    return rc


if __name__ == '__main__':
    sys.exit(main())
