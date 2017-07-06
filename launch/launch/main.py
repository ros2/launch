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

import argparse
import os
import sys

from launch import LaunchDescriptor
from launch.launcher import DefaultLauncher
from launch.loader import load_launch_file


def file_exists(filename):
    return os.path.exists(filename) and os.path.isfile(filename)


def get_launch_arg(arg, separator=":="):
    # Split the arg into a key value pair
    parts = arg.split(separator)
    if len(parts) != 2:
        return None

    return tuple(parts)


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

    launch_files = []
    launch_args = []

    # Validate the list of arguments
    # NOTE: since both the launch files and launch arguments are consecutive
    # lists of arguments, argparse will put all arguments into the launch_file
    # list, and the arg list will be empty. This is because the '+' list is
    # greedy and consumes the remaining arguments. Therefore the arguments
    # need to be validated afterward
    for arg in args.launch_file:
        if len(launch_files) == 0:
            # Found no launch files yet, therefore this must be a launch file
            if file_exists(arg):
                launch_files.append(arg)
            else:
                print("ERROR: '%s' is not a valid launch file" % arg)
                parser.print_help()
                sys.exit(2)
        else:
            # Found at least one launch file before this, need to know
            # if arguments have been found yet
            arg_pair = get_launch_arg(arg)
            if len(launch_args) > 0:
                # Found a previous launch argument, therefore
                # this must also be a launch argument
                if arg_pair is None:
                    print("ERROR: '%s' is not a valid launch argument" % arg)
                    parser.print_help()
                    sys.exit(2)
                else:
                    launch_args.append(arg_pair)
            else:
                # Found no arguments yet, so the argument can
                # be either a launch file, or a launch argument
                if arg_pair is None:
                    # Must be a file
                    if file_exists(arg):
                        launch_files.append(arg)
                    else:
                        print("ERROR: '%s' is not a valid launch file or argument" % arg)
                        parser.print_help()
                        sys.exit(2)
                else:
                    launch_args.append(arg_pair)

    arguments = dict(launch_args)

    launcher = DefaultLauncher()
    for launch_file in launch_files:
        launch_descriptor = LaunchDescriptor()
        load_launch_file(launch_file, launch_descriptor, arguments)
        launcher.add_launch_descriptor(launch_descriptor)
    rc = launcher.launch()
    return rc


if __name__ == '__main__':
    sys.exit(main())
