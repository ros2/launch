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

import sys
import pytest

from datetime import timedelta

from launch import LaunchDescription
from launch.actions import Execute
from launch.actions import RegisterEventHandler
from launch.executables import Process
from launch.event_handlers import OnProcessExited
from launch_testing import TestLaunchService
from launch_testing.actions import AssertOnce

import launch_testing.predicates as predicates
import launch_testing.variables as variables


@pytest.fixture
def launch_service() -> TestLaunchService:
    """
    A test fixture providing a test-aware launch service (i.e. one
    that listens to events fired by `launch_testing.actions.Assert`
    and `launch_testing.actions.Test` actions and reacts accordingly).
    """
    return TestLaunchService()


def test_file_compression(launch_service):
    ld = LaunchDescription()

    prelist_bags_action = Execute(Process(
        cmd='ls *.bag', cwd='/var/log/bags', shell=True
    ), output='screen')
    ld.add_action(prelist_bags_action)

    compression_action = Execute(Process(
        cmd='bzip2 /var/log/bags/*.bag', shell=True
    ), output='screen', prefix='time')
    ld.add_action(compression_action)

    @predicates.custom
    def parse_timedelta(string_value):
        match = re.match(
            '(?P<minutes>\d+)m(?P<seconds>\d\.\d{3})', string_value
        )
        return timedelta(**{
            name: float(value) if value else 0
            for name, value in match.groupdict()
        })

    ld.add_action(
        AssertOnce(
            parse_timedelta(predicates.regex_match(
                variables.Output(compression_action),
                pattern='^real (\d+m\d\.\d{3}s)$', group=1
            )) < timedelta(minutes=1, seconds=30) and
            parse_timedelta(predicates.regex_match(
                variables.Output(compression_action),
                pattern='^sys (\d+m\d\.\d{3}s)$', group=1
            )) < timedelta(seconds=30), timeout=120
        )
    )

    postlist_bags_action = Execute(Process(
        cmd='ls *.bag.bz2', cwd='/var/log/bags', shell=True
    ), output='screen')
    assert_all_bags_compressed = AssertOnce(
        predicates.count(variables.Output(postlist_bags_action))
        == predicates.count(variables.Output(prelist_bags_action))
    )
    ld.add_action(RegisterEventHandler(OnProcessExit(
        target_action=compression_action,
        on_exit=[postlist_bags_action, assert_all_bags_compressed]
    )))

    assert launch_service.run() == 0
