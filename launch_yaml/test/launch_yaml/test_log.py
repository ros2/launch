# Copyright 2020 Open Source Robotics Foundation, Inc.
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

"""Test parsing a `Log` action."""

import io
import textwrap

from launch import LaunchContext
from launch.actions import Log
from launch.actions import LogDebug
from launch.actions import LogError
from launch.actions import LogInfo
from launch.actions import LogWarning
from launch.utilities import perform_substitutions

from parser_no_extensions import load_no_extensions
import pytest


def test_log():
    launch_context = LaunchContext()
    yaml_file = \
        """\
        launch:
        -   log:
                level: INFO
                message: Hello world!
        -   log:
                message: Hello world!
        -   log_info:
                message: Hello world!
        -   log_debug:
                message: Hello world debug!
        -   log_warning:
                message: Hello world warning!
        -   log_error:
                message: Hello world error!
        """
    yaml_file = textwrap.dedent(yaml_file)
    root_entity, parser = load_no_extensions(io.StringIO(yaml_file))
    with pytest.warns(Warning) as record:
        launch_description = parser.parse_description(root_entity)

    assert len(record) == 1

    log = launch_description.entities[0]
    assert isinstance(log, Log)
    assert perform_substitutions(launch_context, log.msg) == 'Hello world!'

    log2 = launch_description.entities[1]
    assert isinstance(log2, Log)
    assert perform_substitutions(launch_context, log2.msg) == 'Hello world!'

    log_info = launch_description.entities[2]
    assert isinstance(log_info, LogInfo)
    assert perform_substitutions(launch_context, log_info.msg) == 'Hello world!'

    log_debug = launch_description.entities[3]
    assert isinstance(log_debug, LogDebug)
    assert perform_substitutions(launch_context, log_debug.msg) == 'Hello world debug!'

    log_warning = launch_description.entities[4]
    assert isinstance(log_warning, LogWarning)
    assert perform_substitutions(launch_context, log_warning.msg) == 'Hello world warning!'

    log_error = launch_description.entities[5]
    assert isinstance(log_error, LogError)
    assert perform_substitutions(launch_context, log_error.msg) == 'Hello world error!'
