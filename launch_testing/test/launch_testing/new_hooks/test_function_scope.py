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


@pytest.fixture(scope='module')
def order():
    return []


@launch_testing.pytest.fixture()
def launch_description():
    return launch.LaunchDescription([
        launch_testing.util.KeepAliveProc(),
        launch_testing.actions.ReadyToTest(),
    ])


# TODO(ivanpauno)
# We cannot get variables from the dictionary returned by launch_description
# because we're removing the fixture before running the tests.
# Maybe we can delete this feature, and use generators/asyncgens.
# We can always get the variables returned by the launch_testing fixture there.
@pytest.mark.launch_testing(fixture=launch_description, shutdown=True)
async def test_after_shutdown(order, launch_service):
    order.append('test_after_shutdown')
    assert launch_service._is_idle()
    assert launch_service.event_loop is None


@pytest.mark.launch_testing(fixture=launch_description)
async def test_case_1(order):
    order.append('test_case_1')
    assert True


@pytest.mark.launch_testing(fixture=launch_description)
def test_case_2(order):
    order.append('test_case_2')
    assert True


@pytest.mark.launch_testing(fixture=launch_description)
def test_case_3(order, launch_service):
    order.append('test_case_3')
    yield
    # assert launch_service._finalized
    order.append('test_case_3[shutdown]')


@pytest.mark.launch_testing(fixture=launch_description)
async def test_case_4(order):
    order.append('test_case_4')
    yield
    order.append('test_case_4[shutdown]')


def test_order(order):
    assert order == [
        'test_after_shutdown',
        'test_case_1',
        'test_case_2',
        'test_case_3',
        'test_case_3[shutdown]',
        'test_case_4',
        'test_case_4[shutdown]',
    ]
