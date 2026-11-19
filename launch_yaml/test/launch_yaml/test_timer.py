# Copyright 2026 Open Source Robotics Foundation, Inc.
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

"""Test parsing a timer action."""

import io
import math
import textwrap

from launch.actions import TimerAction

from parser_no_extensions import load_no_extensions


def test_timer_period_can_be_an_integer():
    yaml_file = \
        """\
        launch:
        -   timer:
                period: 5
                children:
                -   executable:
                        cmd: ls -las
        """
    yaml_file = textwrap.dedent(yaml_file)
    root_entity, parser = load_no_extensions(io.StringIO(yaml_file))
    ld = parser.parse_description(root_entity)
    timer = ld.entities[0]
    assert isinstance(timer, TimerAction)
    assert isinstance(timer.period, float)
    assert math.isclose(timer.period, 5.)
    assert len(timer.actions) == 1
