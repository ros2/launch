import asyncio
import os
import sys

from launch import LaunchDescriptor
from launch.exit_handler import primary_exit_handler
from launch.launcher import DefaultLauncher
from launch.loader import load_launch_file


def test_launch_with_coroutine():
    default_launcher = DefaultLauncher()

    launch_file = os.path.join(os.path.abspath(os.path.dirname(__file__)), 'launch_counter.py')
    launch_descriptor = LaunchDescriptor()
    load_launch_file(launch_file, launch_descriptor, {})

    @asyncio.coroutine
    def coroutine():
        yield from asyncio.sleep(1)
        print('one', file=sys.stderr)
        yield from asyncio.sleep(1)
        print('two', file=sys.stderr)
        yield from asyncio.sleep(1)
        print('three', file=sys.stderr)

    @asyncio.coroutine
    def coroutine2():
        yield from asyncio.sleep(1)
        print('one mississippi', file=sys.stderr)
        yield from asyncio.sleep(1)
        print('two mississippi', file=sys.stderr)
        yield from asyncio.sleep(1)
        print('three mississippi', file=sys.stderr)

    launch_descriptor.add_coroutine(coroutine(), name='coroutine', exit_handler=primary_exit_handler)
    #launch_descriptor.add_coroutine(coroutine2())

    print('launch', file=sys.stderr)
    default_launcher.add_launch_descriptor(launch_descriptor)
    rc = default_launcher.launch()
    print('done', rc, file=sys.stderr)


if __name__ == '__main__':
    test_launch_with_coroutine()
