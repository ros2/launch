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
