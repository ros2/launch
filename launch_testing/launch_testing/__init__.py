from importlib.machinery import SourceFileLoader
import io
import os
import re
import signal

import ament_index_python
from launch.output_handler import LineOutput


class UnmatchedOutputError(BaseException):
    pass


class InMemoryHandler(LineOutput):
    """
    Aggregate data from standard output.

    :param name: Name of the process being tested.
    :param launch_descriptor: :py:obj:`LaunchDescriptor` object that contains the processes in the
        test.
    :param expected_lines: A list of lines to match the output literally or a regular expression
        that will only need one line to match, instead of the entire output.
    :param regex_match: If True, treat the expected_lines as a regular expression in match
        accordingly.
    :param filtered_prefixes: A list of byte strings representing prefixes that will cause output
        lines to be ignored if they start with one of the prefixes. By default lines starting with
        the process ID (`b'pid'`) and return code (`b'rc'`) will be ignored.
    :param filtered_rmw_implementation: RMW implementation for which the output will be ignored
        in addition to the default/`filtered_prefixes`.
    :param exit_on_match: If True, then when its output is matched, this handler
        will terminate; otherwise it will simply keep track of the match.
    :param exact_match: If True, the output received must exactly match the expected output, with
        no unfiltered lines appearing. If False, the output must simply contain the expected
        output, but unfiltered lines will be tolerated. Default value is True.
    :raises: :py:class:`UnmatchedOutputError` if :py:meth:`check` does not find that the output
        matches as expected.
    :raises: :exc:`LookupError` if the `rmw_output_filter` of the `filtered_rmw_implementation`
        cannot be found.
    :raises: :exc:`IOError` if the `rmw_output_filter` of the `filtered_rmw_implementation`
        cannot be opened.
    """

    def __init__(
        self, name, launch_descriptor, expected_lines, regex_match=False,
        filtered_prefixes=None, filtered_rmw_implementation=None, exit_on_match=False,
        exact_match=True
    ):
        super(LineOutput, self).__init__()
        if filtered_prefixes is None:
            self.filtered_prefixes = get_default_filtered_prefixes()
        else:
            self.filtered_prefixes = filtered_prefixes

        if filtered_rmw_implementation:
            rmw_output_filter = get_rmw_output_filter(filtered_rmw_implementation)
            self.filtered_prefixes.extend(rmw_output_filter)

        self.name = name
        self.launch_descriptor = launch_descriptor
        self.expected_lines = expected_lines
        if regex_match:
            self.expected_output = self.expected_lines
            # Add a surrounding capture group
            self.expected_output_captured = \
                b'(?P<launch_testing_capture>' + self.expected_output + b')'
            # Increment any group IDs in the original regex to compensate for the surrounding one
            incrementer = lambda matchobj: '\\' + str(int(matchobj.group(1)) + 1)
            self.expected_output_captured = re.sub(
                (r'\\(\d)'), incrementer, self.expected_output_captured.decode()).encode()
        else:
            self.expected_output = b'\n'.join(self.expected_lines)
            self.expected_output += b'\n'
        self.left_over_stdout = b''
        self.left_over_stderr = b''
        self.stdout_data = io.BytesIO()
        self.stderr_data = io.BytesIO()
        self.regex_match = regex_match
        self.exit_on_match = exit_on_match
        self.exact_match_required = exact_match
        self.matched = False
        self.matched_exactly = False

    def on_stdout_lines(self, lines):
        for line in lines.splitlines():
            # Filter out stdout that comes from underlying DDS implementation
            if any([line.startswith(prefix) for prefix in self.filtered_prefixes]):
                continue
            self.stdout_data.write(line + b'\n')

            received_output = self.stdout_data.getvalue()
            received_lines = received_output.splitlines()

            # Check for literal match
            if not self.regex_match:
                self.matched = self.expected_output in received_output
                self.matched_exactly = self.expected_lines == received_lines

            # Check for regex match
            if self.regex_match:
                self.matched = re.search(self.expected_output_captured, received_output)
                matched_group = self.matched.group('launch_testing_capture') \
                    if self.matched else None
                self.matched_exactly = matched_group == received_output

            if self.matched and self.exit_on_match:
                # We matched and we're in charge; shut myself down
                for td in self.launch_descriptor.task_descriptors:
                    if td.name == self.name:
                        if os.name != 'nt':
                            td.task_state.signals_received.append(signal.SIGINT)
                            td.transport.send_signal(signal.SIGINT)
                        else:
                            td.terminate()
                        return

    def on_stderr_lines(self, lines):
        self.stderr_data.write(lines)

    def get_description(self):
        return 'InMemoryHandler: ' + self.name

    def check(self):
        output_lines = self.stdout_data.getvalue().splitlines()
        success = self.matched_exactly or (self.matched and not self.exact_match_required)
        if not success:
            raise UnmatchedOutputError(
                'Received output does not match expected output.\n' +
                'Received output:\n%r\nExpected output%s:\n%r' %
                (output_lines, ' (regex)' if self.regex_match else '', self.expected_lines))


def get_default_filtered_prefixes():
    return [
        b'pid', b'rc',
    ]


def get_rmw_output_filter(rmw_implementation):
    prefix_with_resource = ament_index_python.has_resource(
        'rmw_output_filter', rmw_implementation)
    if not prefix_with_resource:
        return []

    rmw_output_filter, _ = ament_index_python.get_resource('rmw_output_filter', rmw_implementation)
    additional_filtered_prefixes = [
        str.encode(l) for l in rmw_output_filter.splitlines()]
    return additional_filtered_prefixes


def create_handler(
    name, launch_descriptor, output_file, exit_on_match=False, filtered_prefixes=None,
    filtered_rmw_implementation=None, exact_match=True
):
    literal_file = output_file + '.txt'
    if os.path.isfile(literal_file):
        with open(literal_file, 'rb') as f:
            expected_output = f.read().splitlines()
        return InMemoryHandler(
            name, launch_descriptor, expected_output, regex_match=False,
            exit_on_match=exit_on_match, filtered_prefixes=filtered_prefixes,
            filtered_rmw_implementation=filtered_rmw_implementation, exact_match=exact_match)
    regex_file = output_file + '.regex'
    if os.path.isfile(regex_file):
        with open(regex_file, 'rb') as f:
            expected_output = f.read()
        return InMemoryHandler(
            name, launch_descriptor, expected_output, regex_match=True,
            exit_on_match=exit_on_match, filtered_prefixes=filtered_prefixes,
            filtered_rmw_implementation=filtered_rmw_implementation, exact_match=exact_match)
    py_file = output_file + '.py'
    if os.path.isfile(py_file):
        checker_module = SourceFileLoader(
            'checker_module', py_file).load_module()
        return checker_module.CheckerHandler(name, launch_descriptor)
