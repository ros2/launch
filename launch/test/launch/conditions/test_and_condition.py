# Copyright 2022 Open Source Robotics Foundation, Inc.
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
# POSSIBILITY OF SUCH DAMAGE.

from launch.conditions import AndCondition
from launch.conditions import IfCondition

import pytest


def test_and_condition():
    """Test AndCondition class."""
    class MockLaunchContext:

        def perform_substitution(self, substitution):
            return substitution.perform(self)

    lc = MockLaunchContext()
    test_cases = [
        (['true', 'True', 'TRUE', '1'], True),
        (('true', 'True', 'TRUE', '1'), True),
        (['true', 'false'], False),
        (['false', 'false'], False),
        (['true', IfCondition('true')], True),
        (['true', IfCondition('false')], False),
    ]

    for conditions, expected in test_cases:
        assert AndCondition(conditions).evaluate(lc) is expected

    with pytest.raises(TypeError):
        AndCondition('true').evaluate(lc)

    with pytest.raises(TypeError):
        AndCondition(['true']).evaluate(lc)
