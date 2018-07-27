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

"""Module for the IncludeLaunchDescription action."""

from typing import List

from ..action import Action
from ..launch_context import LaunchContext
from ..launch_description_entity import LaunchDescriptionEntity
from ..launch_description_source import LaunchDescriptionSource


class IncludeLaunchDescription(Action):
    """Action that includes a launch description source and yields its entities when visited."""

    def __init__(self, launch_description_source: LaunchDescriptionSource) -> None:
        """Constructor."""
        super().__init__()
        self.__launch_description_source = launch_description_source

    @property
    def launch_description_source(self) -> LaunchDescriptionSource:
        """Getter for self.__launch_description_source."""
        return self.__launch_description_source

    def visit(self, context: LaunchContext) -> List[LaunchDescriptionEntity]:
        """Override visit to return an Entity rather than an action."""
        return [self.__launch_description_source.launch_description]
