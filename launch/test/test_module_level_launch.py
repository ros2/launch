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
    print('setup', file=sys.stderr)
    default_launcher = DefaultLauncher()

    launch_file = os.path.join(os.path.abspath(os.path.dirname(__file__)), 'launch_counter.py')
    launch_descriptor = LaunchDescriptor()
    load_launch_file(launch_file, launch_descriptor, {})
    default_launcher.add_launch_descriptor(launch_descriptor)

    async_launcher = AsynchronousLauncher(default_launcher)
    print('setup spawning', file=sys.stderr)
    async_launcher.start()
    print('setup spawned', file=sys.stderr)


def teardown():
    global async_launcher
    print('teardown', file=sys.stderr)
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


def handleSIGCHLD(signum, frame):
    print('handleSIGCHLD', file=sys.stderr)


if __name__ == '__main__':
    setup()
    teardown()
