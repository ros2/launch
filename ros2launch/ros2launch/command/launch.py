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

from argparse import REMAINDER
import os

from ament_index_python.packages import PackageNotFoundError
from ros2cli.command import CommandExtension
from ros2launch.api import get_share_file_path_from_package
from ros2launch.api import launch_a_python_launch_file
from ros2launch.api import LaunchFileNameCompleter
from ros2launch.api import MultipleLaunchFilesError
from ros2launch.api import print_a_python_launch_file
from ros2pkg.api import package_name_completer


class LaunchCommand(CommandExtension):
    """Run a package specific executable."""

    def add_arguments(self, parser, cli_name):
        """Add arguments to argparse."""
        parser.add_argument(
            '-d', '--debug', default=False, action='store_true',
            help='Put the launch system in debug mode, provided more verbose output.')
        parser.add_argument(
            '-p', '--print', '--print-description', default=False, action='store_true',
            help='Print the launch description to the console without launching it.')
        arg = parser.add_argument(
            'package_name',
            help='Name of the ROS package which contains the launch file')
        arg.completer = package_name_completer
        arg = parser.add_argument(
            'launch_file_name',
            nargs='?',
            help='Name of the launch file')
        arg.completer = LaunchFileNameCompleter()
        parser.add_argument(
            'argv', nargs=REMAINDER,
            help='Pass arbitrary arguments to the launch file')

    def main(self, *, parser, args):
        """Entry point for CLI program."""
        if args.launch_file_name is None:
            # TODO(wjwwood): figure out how to have argparse and argcomplete
            # handle this, for now, hidden feature.
            if os.path.exists(args.package_name):
                path = args.package_name
            else:
                return 'No launch file supplied'
        else:
            try:
                path = get_share_file_path_from_package(
                    package_name=args.package_name,
                    file_name=args.launch_file_name)
            except PackageNotFoundError as exc:
                raise RuntimeError(
                    "Package '{args.package_name}' not found: {exc}".format_map(locals()))
            except (FileNotFoundError, MultipleLaunchFilesError) as exc:
                raise RuntimeError(str(exc))
        if args.print:
            return print_a_python_launch_file(python_launch_file_path=path)
        else:
            return launch_a_python_launch_file(
                python_launch_file_path=path,
                launch_file_arguments=args.argv,
                debug=args.debug
            )
