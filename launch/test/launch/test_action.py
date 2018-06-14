# Copyright 2018 Open Source Robotics Foundation, Inc.
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

"""Tests for the Action class."""

from launch import Action


def test_action_constructors():
    """Test the constructors for Action class."""
    Action()


def test_action_methods():
    """Test the moethods of the Action class."""
    class MockLaunchContext:
        ...

    action = Action()
    assert 'Action' in action.describe()
    assert isinstance(action.describe_sub_entities(), list)
    assert isinstance(action.describe_conditional_sub_entities(), list)
    assert action.visit(MockLaunchContext()) is None
    assert action.get_asyncio_future() is None

    class CustomAction(Action):

        def __init__(self):
            super().__init__()
            self.execute_called = False

        def execute(self, context):
            self.execute_called = True

    custom_action = CustomAction()
    assert 'CustomAction' in custom_action.describe()
    assert isinstance(custom_action.describe_sub_entities(), list)
    assert isinstance(custom_action.describe_conditional_sub_entities(), list)
    assert custom_action.visit(MockLaunchContext()) is None
    assert custom_action.execute_called is True
    assert custom_action.get_asyncio_future() is None
