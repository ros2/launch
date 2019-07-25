# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import re


def get_default_filtered_prefixes():
    return [
        'pid', 'rc',
    ]


def get_default_filtered_patterns():
    return []


def basic_output_filter(
    filtered_prefixes=None,
    filtered_patterns=None,
):
    """
    Create a line filtering function to help output testing.

    :param filtered_prefixes: A list of byte strings representing prefixes that will cause
    output lines to be ignored if they start with one of the prefixes. By default lines
    starting with the process ID (`'pid'`) and return code (`'rc'`) will be ignored.
    :param filtered_patterns: A list of byte strings representing regexes that will cause
    output lines to be ignored if they match one of the regexes.
    """
    if filtered_prefixes is None:
        filtered_prefixes = get_default_filtered_prefixes()
    if filtered_patterns is None:
        filtered_patterns = get_default_filtered_patterns()
    filtered_patterns = list(map(re.compile, filtered_patterns))

    def _filter(output):
        filtered_output_lines = []
        for line in output.splitlines():
            # Filter out stdout that comes from underlying DDS implementation
            # Note: we do not currently support matching filters across multiple stdout lines.
            if any(line.startswith(prefix) for prefix in filtered_prefixes):
                continue
            if any(pattern.match(line) for pattern in filtered_patterns):
                continue
            filtered_output_lines.append(line)
        filtered_output = os.linesep.join(filtered_output_lines)
        if filtered_output and output.endswith(os.linesep):
            filtered_output += os.linesep
        return filtered_output
    return _filter


def expected_output_from_file(path):
    """
    Get expected output lines from a file.

    :param path: path w/o extension of either a .txt file containing the lines
    to be matched or a .regex file containing patterns to be searched for.
    """
    literal_file = path + '.txt'
    if os.path.isfile(literal_file):
        with open(literal_file, 'r') as f:
            return f.read().splitlines()

    regex_file = path + '.regex'
    if os.path.isfile(regex_file):
        with open(regex_file, 'r') as f:
            return [re.compile(regex) for regex in f.read().splitlines()]

    raise RuntimeError('could not find output check file: {}'.format(path))
