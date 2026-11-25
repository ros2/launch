# Copyright 2025 Open Source Robotics Foundation, Inc.
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

"""Tests for the ForEachVar substitution."""

from launch import LaunchContext
from launch.substitutions import ForEachVar
from launch.substitutions import TextSubstitution
import pytest


def test_for_each_var():
    context = LaunchContext()

    # No value available, no default value provided
    with pytest.raises(RuntimeError):
        ForEachVar('name').perform(context)

    # No value available, default value provided
    value = ForEachVar('name', default_value='default').perform(context)
    assert value == 'default'

    # Value available
    context.extend_locals({ForEachVar.get_local_arg_name('name'): 'some_value'})
    value = ForEachVar(TextSubstitution(text='name')).perform(context)
    assert value == 'some_value'
