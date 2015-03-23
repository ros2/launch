import argparse
import os
import sys

from launch import LaunchDescriptor
from launch.launcher import DefaultLauncher
from launch.loader import load_launch_file


def file_exists(filename):
    if not os.path.exists(filename) or not os.path.isfile(filename):
        raise argparse.ArgumentError("'%s' does not exist" % filename)
    return filename


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(
        description='Launch the processes specified in a launch file.')
    parser.add_argument(
        'launch_file',
        type=file_exists,
        nargs='+',
        help='The launch file.')
    args = parser.parse_args(argv)

    arguments = {}

    launcher = DefaultLauncher()
    for launch_file in args.launch_file:
        launch_descriptor = LaunchDescriptor()
        load_launch_file(launch_file, launch_descriptor, arguments)
        launcher.add_launch_descriptor(launch_descriptor)
    rc = launcher.launch()
    return rc


if __name__ == '__main__':
    sys.exit(main())
