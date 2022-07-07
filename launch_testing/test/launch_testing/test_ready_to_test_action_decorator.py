# Copyright 2022 Apex.AI, Inc.
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

import launch_testing


def test_ready_to_test_action_attribute():

    @launch_testing.ready_to_test_action_timeout(10)
    def fake_test_description(arg):
        pass  # pragma: no cover

    assert hasattr(fake_test_description, '__ready_to_test_action_timeout__')
    assert getattr(fake_test_description, '__ready_to_test_action_timeout__') == 10


def test_parametrize_and_ready_to_test_action_attribute():

    @launch_testing.parametrize('val', [1, 2, 3])
    @launch_testing.ready_to_test_action_timeout(10)
    def fake_test_description(arg):
        pass  # pragma: no cover

    assert hasattr(fake_test_description, '__parametrized__')
    assert hasattr(fake_test_description, '__ready_to_test_action_timeout__')
    assert getattr(fake_test_description, '__ready_to_test_action_timeout__') == 10


def test_ready_to_test_action_and_parametrize_attribute():

    @launch_testing.ready_to_test_action_timeout(10)
    @launch_testing.parametrize('val', [1, 2, 3])
    def fake_test_description(arg):
        pass  # pragma: no cover

    assert hasattr(fake_test_description, '__parametrized__')
    assert hasattr(fake_test_description, '__ready_to_test_action_timeout__')
    assert getattr(fake_test_description, '__ready_to_test_action_timeout__') == 10

