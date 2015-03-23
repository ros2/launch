import os
import sys

from launch.exit_handler import IgnoreExitHandler


def launch(launch_descriptor, argv):
    counter_file = os.path.join(os.path.abspath(os.path.dirname(__file__)), 'counter.py')

    ld = launch_descriptor
    ld.add_process(
        cmd=[sys.executable, '-u', counter_file, '--limit', '5', '--sleep', '0.5'],
        name='foo',
        exit_handler=IgnoreExitHandler(),
    )
