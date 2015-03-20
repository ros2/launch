import os
import sys
from tempfile import NamedTemporaryFile

from launch.exit_handler import IgnoreExitHandler
from launch.exit_handler import RestartExitHandler
from launch.output_handler import FileOutput
from launch.output_handler import ConsoleOutput


def launch(launch_descriptor):
    counter_file = os.path.join(os.path.abspath(os.path.dirname(__file__)), 'counter.py')

    with NamedTemporaryFile(mode='w', prefix='foo_', delete=False) as h:
        foo_filename = h.name
    with NamedTemporaryFile(mode='w', prefix='baz-err_', delete=False) as h:
        baz_filename = h.name

    ld = launch_descriptor
    ld.add_process(
        cmd=[sys.executable, '-u', counter_file, '--limit', '9', '--sleep', '0.5'],
        name='foo',
        output_handlers=[FileOutput(filename=foo_filename)],
        exit_handler=RestartExitHandler(),
    )
    ld.add_process(
        cmd=[sys.executable, '-u', counter_file, '--limit', '10', '--sleep', '0.25'],
        name='bar',
        env=None,
        exit_handler=IgnoreExitHandler(),
    )
    ld.add_process(
        cmd=[sys.executable, '-u', counter_file, '--limit', '5', '--sleep', '1'],
        name='baz',
        output_handlers=[FileOutput(filename_stderr=baz_filename)],
    )
    ld.add_process(
        cmd=[sys.executable, '-u', counter_file, '--sleep', '0.5'],
        name='qux',
        output_handlers=[ConsoleOutput(stderr_only=True)],
    )
