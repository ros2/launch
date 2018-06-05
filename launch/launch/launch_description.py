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

"""Module for LaunchDescription class."""

from typing import Iterable
from typing import List
from typing import Optional

from .action import Action
from .launch_context import LaunchContext
from .launch_description_entity import LaunchDescriptionEntity


class LaunchDescription(LaunchDescriptionEntity):
    """
    Description of a launch-able system.

    The description is expressed by a collection of entities which represent
    the system architect's intentions.
    """

    def __init__(
        self,
        initial_entities: Optional[Iterable[LaunchDescriptionEntity]] = None
    ) -> None:
        """Constructor."""
        self.__entities = list(initial_entities) if initial_entities is not None else []

    def visit(self, context: LaunchContext) -> Optional[List[LaunchDescriptionEntity]]:
        """Override visit from LaunchDescriptionEntity to visit contained entities."""
        return self.__entities

    @property
    def entities(self) -> List[LaunchDescriptionEntity]:
        """Getter for the entities."""
        return self.__entities

    def add_entity(self, entity: LaunchDescriptionEntity) -> None:
        """Add an entity to the LaunchDescription."""
        self.__entities.append(entity)

    def add_action(self, action: Action) -> None:
        """Add an action based on the given ActionType and arguments."""
        self.add_entity(action)
