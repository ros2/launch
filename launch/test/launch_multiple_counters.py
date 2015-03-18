#!/usr/bin/env python3

import os
import sys

from launch.exit_handler import IgnoreExitHandler
from launch.exit_handler import RestartExitHandler
from launch.launcher import DefaultLauncher
from launch.output_handler import FileOutput
from launch.output_handler import ConsoleOutput


def main():
    counter_file = os.path.join(os.path.dirname(__file__), 'counter.py')

    launcher = DefaultLauncher(name_prefix='')
    launcher.add_process(
        cmd=[sys.executable, '-u', counter_file, '--limit', '9', '--sleep', '0.5'],
        name='foo',
        output_handlers=[FileOutput(filename='/tmp/foo.log')],
        exit_handler=RestartExitHandler(),
    )
    launcher.add_process(
        cmd=[sys.executable, '-u', counter_file, '--limit', '10', '--sleep', '0.25'],
        name='bar',
        env=None,
        exit_handler=IgnoreExitHandler(),
    )
    launcher.add_process(
        cmd=[sys.executable, '-u', counter_file, '--limit', '5', '--sleep', '1'],
        name='baz',
        output_handlers=[FileOutput(filename_stderr='/tmp/baz-err.log')],
    )
    launcher.add_process(
        cmd=[sys.executable, '-u', counter_file, '--sleep', '0.5'],
        name='qux',
        output_handlers=[ConsoleOutput(stderr_only=True)],
    )
    rc = launcher.launch()
    return rc


if __name__ == '__main__':
    sys.exit(main())
