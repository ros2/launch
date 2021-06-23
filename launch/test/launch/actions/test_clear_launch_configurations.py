# Copyright 2021 Open Source Robotics Foundation, Inc.
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

"""Tests for the ClearLaunchConfigurations action class."""

from launch import LaunchContext
from launch.actions import ClearLaunchConfigurations


def test_clear_launch_configurations_constructors():
    """Test the constructors for ClearLaunchConfigurations class."""
    ClearLaunchConfigurations()
    ClearLaunchConfigurations(launch_configurations_to_not_be_cleared=[])
    ClearLaunchConfigurations(launch_configurations_to_not_be_cleared=['arg1', 'arg2', 'arg3'])


def test_clear_launch_configurations_execute():
    """Test the execute() of the ClearLaunchConfigurations class."""
    # Clear all existing launch configurations without initializer for
    # launch_configurations_to_not_be_cleared
    lc1 = LaunchContext()
    assert len(lc1.launch_configurations) == 0
    lc1.launch_configurations['foo'] = 'FOO'
    lc1.launch_configurations['bar'] = 'BAR'
    assert len(lc1.launch_configurations) == 2
    ClearLaunchConfigurations().visit(lc1)
    assert len(lc1.launch_configurations) == 0

    # Clear all existing launch configurations with initializer for
    # launch_configurations_to_not_be_cleared = None
    lc2 = LaunchContext()
    assert len(lc2.launch_configurations) == 0
    lc2.launch_configurations['foo'] = 'FOO'
    lc2.launch_configurations['bar'] = 'BAR'
    assert len(lc2.launch_configurations) == 2
    ClearLaunchConfigurations(launch_configurations_to_not_be_cleared=None).visit(lc2)
    assert len(lc2.launch_configurations) == 0

    # Clear all existing launch configurations with initializer for
    # launch_configurations_to_not_be_cleared = []
    lc3 = LaunchContext()
    assert len(lc3.launch_configurations) == 0
    lc3.launch_configurations['foo'] = 'FOO'
    lc3.launch_configurations['bar'] = 'BAR'
    assert len(lc3.launch_configurations) == 2
    ClearLaunchConfigurations(launch_configurations_to_not_be_cleared=[]).visit(lc3)
    assert len(lc3.launch_configurations) == 0

    # Exclude a launch configuration from being cleared
    lc4 = LaunchContext()
    assert len(lc4.launch_configurations) == 0
    lc4.launch_configurations['foo'] = 'FOO'
    lc4.launch_configurations['bar'] = 'BAR'
    assert len(lc4.launch_configurations) == 2
    ClearLaunchConfigurations(launch_configurations_to_not_be_cleared=['foo']).visit(lc4)
    assert len(lc4.launch_configurations) == 1
    assert 'bar' not in lc4.launch_configurations.keys()

    # Try to exclude a launch configuration from being cleared that doesn't exist
    lc5 = LaunchContext()
    assert len(lc5.launch_configurations) == 0
    lc5.launch_configurations['foo'] = 'FOO'
    lc5.launch_configurations['bar'] = 'BAR'
    assert len(lc5.launch_configurations) == 2
    ClearLaunchConfigurations(launch_configurations_to_not_be_cleared=['baz']).visit(lc5)
    assert len(lc5.launch_configurations) == 0
