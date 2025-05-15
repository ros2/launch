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

"""Tests for the PathJoinSubstitution substitution class."""

import os

from launch import LaunchContext
from launch.substitutions import PathJoinSubstitution, PathSubstitution
from launch.substitutions import TextSubstitution


def test_path_join():
    context = LaunchContext()

    path = ['asd', 'bsd', 'cds']
    sub = PathJoinSubstitution(path)
    assert sub.perform(context) == os.path.join(*path)

    path = ['path', ['to'], ['my_', TextSubstitution(text='file'), '.yaml']]
    sub = PathJoinSubstitution(path)
    assert sub.perform(context) == os.path.join('path', 'to', 'my_file.yaml')

    sub = PathSubstitution('some') / 'path'
    sub = sub / PathJoinSubstitution(['to', 'some', 'dir'])
    sub = sub / (TextSubstitution(text='my_model'), '.xacro')
    assert sub.perform(context) == os.path.join(
        'some', 'path', 'to', 'some', 'dir', 'my_model.xacro')
