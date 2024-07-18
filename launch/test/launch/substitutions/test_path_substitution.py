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

"""Tests for the PathSubstitution substitution class."""

import os
from pathlib import Path

from launch.substitutions import PathSubstitution


def test_path_join():
    path = Path('asd') / 'bsd' / 'cds'
    sub = PathSubstitution(path=path)
    assert sub.perform(None) == os.path.join('asd', 'bsd', 'cds')
