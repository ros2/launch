import os
import sys
import tempfile

from launch import LaunchDescriptor
from launch.exit_handler import ignore_exit_handler
from launch.exit_handler import primary_ignore_returncode_exit_handler
from launch.launcher import DefaultLauncher
from launch.output_handler import ConsoleOutput
from launch_testing import create_handler


def setup():
    os.environ['OSPL_VERBOSITY'] = '8'  # 8 = OS_NONE


def test_executable():
    output_handlers = []

    launch_descriptor = LaunchDescriptor()

    num_executables = 2
    executable_command = [
        sys.executable, '-u',
        os.path.join(os.path.abspath(os.path.dirname(__file__)), 'matching.py')]

    tempdir = tempfile.mkdtemp()
    output_file = os.path.join(tempdir, 'testfile')
    full_output_file = output_file + '.txt'
    with open(full_output_file, 'w+') as f:
        f.write('this is line 1\nthis is line b\n')

    for i in range(num_executables):
        name = 'test_executable_' + str(i)
        # The last executable is taken to be the test program (the one whose
        # output check can make the decision to shut everything down)
        if i == (num_executables - 1):
            exit_handler = primary_ignore_returncode_exit_handler
            exit_on_match = True
            exact_match = True
        else:
            exit_handler = ignore_exit_handler
            exit_on_match = False
            exact_match = False  # TODO(dhood): finer ignoring of SIGINT

        handler = create_handler(
            name, launch_descriptor, output_file, exact_match=exact_match,
            exit_on_match=exit_on_match)
        assert handler, 'Cannot find appropriate handler for %s' % output_file
        output_handlers.append(handler)
        launch_descriptor.add_process(
            cmd=executable_command,
            name=name,
            exit_handler=exit_handler,
            output_handlers=[ConsoleOutput(), handler],
        )

    launcher = DefaultLauncher()
    launcher.add_launch_descriptor(launch_descriptor)
    rc = launcher.launch()

    assert rc == 0, \
        "The launch file failed with exit code '" + str(rc) + "'. " \
        'Maybe the client did not receive any messages?'

    for handler in output_handlers:
        handler.check()


if __name__ == '__main__':
    test_executable()
