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

"""
Some example tests for ROS-aware launch descriptions.
"""

import sys
import pytest

from launch import LaunchDescription
from launch.events.process import ProcessStarted
from launch.launch_description_sources import \
    get_launch_description_from_python_launch_file
from launch_ros import get_default_launch_description
from launch_ros.actions import Node
import launch_ros.events.lifecycle as lifecyle

from launch_testing import TestLaunchService
from launch_testing.actions import PyTest
from launch_testing.actions import GTest
from launch_testing.actions import Assert
from launch_testing.actions import AssertSequenceOnce

import launch_testing.predicates as predicates
import launch_testing.variables as variables


@pytest.fixture
def ros_test_launch_service() -> TestLaunchService:
    """
    A test fixture providing a test-aware launch service (i.e. one
    that listens to events fired by `launch_testing.actions.Assert`
    and `launch_testing.actions.Test` actions and reacts accordingly).
    """
    ls = TestLaunchService()
    # TODO(hidmic): consider integrating ROS specific actions and the
    # launch service in some way to ease launch file composition without
    # repetition).
    ls.include_launch_description(
        get_default_launch_description(prefix_output_with_name=False)
    )
    return ls


@pytest.fixture
def pub_sub_launch_description() -> LaunchDescription:
    """
    A test fixture providing the basic pub/sub launch description.
    """
    # TODO(hidmic): implement functionality to lookup launch files
    # on a per package basis
    launch_path = get_launch_path(
        package_name='launch_ros', launch_file_path='pub_sub_launch.py'
    )
    return get_launch_description_from_python_launch_file(launch_path)


def test_pub_sub_launch(ros_test_launch_service, pub_sub_launch_description):
    """
    Tests an existing simple pub/sub system.
    """
    # Launches a pytest that also happens to be a node with
    # a 30s timeout.
    pub_sub_launch_description.add_action(
        PyTest(path='pub_test.py', timeout=30)
    )
    # Launches a gtest that also happens to be a node with
    # a 30s timeout.
    pub_sub_launch_description.add_action(
        GTest(path='sub_test', timeout=30)
    )

    ros_test_launch_service.include_launch_description(pub_sub_launch_description)

    # TODO(hidmic): implement launch_testing specific pytest plugin to aggregate
    # all test result information at the launch system-level and below.
    assert ros_test_launch_service.run() == 0


@pytest.fixture
def lifecycle_pub_sub_launch_description() -> LaunchDescription:
    """
    A test fixture providing the lifecycle pub/sub launch description.
    """
    # TODO(hidmic): implement functionality to lookup launch files
    # on a per package basis
    launch_path = get_launch_path(
        package_name='launch_ros',
        launch_file_path='lifecycle_pub_sub_launch.py'
    )
    return get_launch_description_from_python_launch_file(launch_path)


def test_lifecycle_pub_sub_launch(ros_test_launch_service,
                                  lifecycle_pub_sub_launch_description):
    """
    Tests an existing lifecycle pub/sub nodes setup for execution
    in proper sequence.
    """
    # Asserts that a lifecycle.ChangeState event within a 30s window
    # is followed by a process.ProcessStarted event within a 10s window
    # once (40s total timeout).
    lifecycle_pub_sub_launch_description.add_action(
        AssertSequenceOnce([
            variables.EventEmitted(
                event_type=lifecycle.ChangeState
            ),
            variables.EventEmitted(
                event_type=ProcessStarted
            )
        ], timeout=[30, 10])
    )

    ros_test_launch_service.include_launch_description(
        lifecycle_pub_sub_launch_description
    )
    # TODO(hidmic): implement launch_testing specific pytest plugin to
    # aggregate all test result information at the launch system-level
    # and below.
    assert ros_test_launch_service.run() == 0


def test_pub_sub(ros_test_launch_service):
    """
    Tests a basic pub/sub setup for proper output matching.
    """
    ld = launch.LaunchDescription()
    talker_node_action = launch_ros.actions.Node(
        package='demo_nodes_cpp',
        node_executable='talker',
        remappings=[('chatter', 'my_chatter')],
        output='screen'
    )
    ld.add_action(talker_node_action)
    listener_node_action = launch_ros.actions.Node(
        package='demo_nodes_py',
        node_executable='listener',
        remappings=[('chatter', 'my_chatter')],
        output='screen'
    )
    ld.add_action(listener_node_action)
    # Asserts that not a single talker screen output
    # line contains the ERROR string throughout the duration
    # of the test (i.e. it completes when all other tests
    # have completed). The assertion is strict (i.e. failure
    # on a single output event means failure altogether).
    ld.add_action(
        Assert(not predicates.regex_match(
            variables.Output(talker_node_action),
            pattern='.*ERROR.*', flags=None
        ), timeout=None, strict=True)
    )
    # Asserts that a the listener outputs 'I heard...'
    # at least once in the first 10s of the test. The
    # assertion is not strict (i.e. failure on a single
    # output event does not mean failure altogether).
    ld.add_action(
        Assert(predicates.count(predicates.regex_match(
            variables.Output(listener_node_action),
            pattern='I heard.*', flags=None
        )) > 0, message='No message received!', timeout=10)
    )
    ros_test_launch_service.include_launch_description(ld)

    # TODO(hidmic): implement launch_testing specific pytest plugin to
    # aggregate all test result information at the launch system-level
    # and below.
    assert ros_test_launch_service.run() == 0
