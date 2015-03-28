import os
import sys
from tempfile import NamedTemporaryFile

from launch import LaunchDescriptor
from launch.exit_handler import ignore_exit_handler
from launch.exit_handler import restart_exit_handler
from launch.loader import load_launch_file
from launch.output_handler import FileOutput
from launch.output_handler import ConsoleOutput


def launch(launch_descriptor, argv):
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
        exit_handler=restart_exit_handler,
    )
    ld.add_process(
        cmd=[sys.executable, '-u', counter_file, '--limit', '10', '--sleep', '0.25'],
        name='bar',
        env=None,
        exit_handler=ignore_exit_handler,
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

    other_launch_file = os.path.join(
        os.path.abspath(os.path.dirname(__file__)), 'launch_counter.py')
    sub_ld = LaunchDescriptor()
    load_launch_file(other_launch_file, sub_ld, {})
    # namespace all processes from other launch file
    for d in sub_ld.process_descriptors:
        d.name = 'sub.' + d.name
        ld.process_descriptors.append(d)
