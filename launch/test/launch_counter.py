import os
import sys

from launch.exit_handler import IgnoreExitHandler
from launch.output_handler import FileOutput


def launch(launch_descriptor, argv):
    counter_file = os.path.join(os.path.abspath(os.path.dirname(__file__)), 'counter.py')

    ld = launch_descriptor
    ld.add_process(
        cmd=[sys.executable, '-u', counter_file, '--limit', '25', '--sleep', '0.5'],
        name='foo',
        #output_handlers=[FileOutput(filename='/tmp/foo.log')],
        exit_handler=IgnoreExitHandler(),
    )
