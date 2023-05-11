# Copyright 2023 Open Source Robotics Foundation, Inc.
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

"""Tests the FileContent substitution."""

import pathlib

from launch.launch_context import LaunchContext
from launch.substitutions import FileContent
from launch.substitutions.substitution_failure import SubstitutionFailure

import pytest


@pytest.fixture(scope='module')
def files():
    this_dir = pathlib.Path(__file__).parent

    files = {
        'foo': str(this_dir / 'test_file_content' / 'foo.txt'),
        'bar': str(this_dir / 'test_file_content' / 'bar.txt'),
    }

    return files


def test_file_content(files):
    """Test a simple file."""
    context = LaunchContext()
    file_content = FileContent(files['foo'])
    output = file_content.perform(context)
    assert output == 'Foo\n'


def test_missing_command_raises(files):
    """Test that a file that doesn't exist raises."""
    context = LaunchContext()
    file_content = FileContent(files['bar'])
    with pytest.raises(SubstitutionFailure) as ex:
        file_content.perform(context)
    ex.match('File not found: .*')
