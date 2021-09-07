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

import launch
import launch_testing

import pytest


@launch_testing.pytest.fixture(scope='module')
def ld():
    return launch.LaunchDescription([launch_testing.actions.ReadyToTest()])


@pytest.mark.launch_testing(fixture=ld)
async def test_case_1():
    assert True


@pytest.mark.launch_testing(fixture=ld)
def test_case_2():
    assert True
