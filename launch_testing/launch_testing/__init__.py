from importlib.machinery import SourceFileLoader
import io
import os
import re
from launch.output_handler import LineOutput


class InMemoryHandler(LineOutput):

    """Aggregate data from standard output.

    @param name: Name of the process being tested.
    @param launch_descriptor: L{LaunchDescriptor} object that contains the processes in the test.
    @param expected_lines: A list of lines to match the output literally or a regular expression
        that will only need one line to match, instead of the entire output.
    @param regex_match: If true, treat the expected_lines as a regular expression in match
        accordingly.
    @param filtered_prefixes: A list of prefixes that will be ignored. By default the output of
        RTI Connext will be ignored.
    """

    # TODO(esteve): This requires internal knowledge about the rmw implementations available.
    def __init__(
        self, name, launch_descriptor, expected_lines, regex_match=False,
        filtered_prefixes=None
    ):
        super(LineOutput, self).__init__()
        if filtered_prefixes is None:
            self.filtered_prefixes = [
                b'pid', b'rc',
                b'RTI Data Distribution Service Evaluation License issued to',
                b'Expires on']
        else:
            self.filtered_prefixes = filtered_prefixes
        self.name = name
        self.launch_descriptor = launch_descriptor
        self.expected_lines = expected_lines
        self.killed = False
        self.left_over_stdout = b''
        self.left_over_stderr = b''
        self.stdout_data = io.BytesIO()
        self.stderr_data = io.BytesIO()
        self.regex_match = regex_match
        self.matched = False

    def on_stdout_lines(self, lines):
        if self.killed:
            return

        for line in lines.splitlines():
            # Filter out stdout that comes from underlying DDS implementations
            if any([line.startswith(prefix) for prefix in self.filtered_prefixes]):
                continue
            self.stdout_data.write(line + b'\n')

            # Are we ready to quit?
            if self.regex_match:
                self.matched = re.match(self.expected_lines[0], line)
            else:
                output_lines = self.stdout_data.getvalue().splitlines()
                self.matched = output_lines == self.expected_lines
            if self.matched:
                # We have enough output to compare; shut down my child
                for td in self.launch_descriptor.task_descriptors:
                    if td.name == self.name:
                        td.terminate()
                        self.killed = True
                        break

    def on_stderr_lines(self, lines):
        self.stderr_data.write(lines)

    def get_description(self):
        return 'InMemoryHandler: ' + self.name

    def check(self):
        output_lines = self.stdout_data.getvalue().splitlines()
        assert self.matched, \
            'Example output (%r) does not match expected output (%r)' % \
            (output_lines, self.expected_lines)


def create_handler(name, launch_descriptor, output_file):
    literal_file = output_file + '.txt'
    if os.path.isfile(literal_file):
        with open(literal_file, 'rb') as f:
            expected_output = f.read().splitlines()
        return InMemoryHandler(name, launch_descriptor, expected_output, regex_match=False)
    regex_file = output_file + '.regex'
    if os.path.isfile(regex_file):
        with open(regex_file, 'rb') as f:
            expected_output = f.read().splitlines()
        return InMemoryHandler(name, launch_descriptor, expected_output, regex_match=True)
    py_file = output_file + '.py'
    if os.path.isfile(py_file):
        checker_module = SourceFileLoader(
            'checker_module', py_file).load_module()
        return checker_module.CheckerHandler(name, launch_descriptor)
