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

"""Tests for the EventHandler class."""

from launch import EventHandler
from launch import LaunchContext
from launch import LaunchDescriptionEntity
from launch import SomeActionsType_types_tuple


def test_event_handler_constructors():
    """Test the constructors for EventHandler class."""
    EventHandler(matcher=lambda event: False)
    EventHandler(matcher=lambda event: False, entities=[LaunchDescriptionEntity])


def test_event_handler_matches_and_handle():
    """Test the matches and handle methods for the EventHandler class."""
    class MockEvent:
        ...

    eh = EventHandler(matcher=lambda event: True, entities=[LaunchDescriptionEntity])
    assert eh.matches(MockEvent()) is True
    mock_event = MockEvent()
    context = LaunchContext()
    entities = eh.handle(mock_event, context)
    assert isinstance(entities, SomeActionsType_types_tuple)
    assert len(entities) == 1
    assert context.locals.event == mock_event
