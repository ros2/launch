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
"""Module for the AbstractLaunchDescriptionSource class."""

import abc
from typing import Optional

from .launch_context import LaunchContext
from .launch_description import LaunchDescription


class AbstractLaunchDescriptionSource(abc.ABC):
    """Encapsulation of a launch description, where it comes from, and how it was generated.
    This is an abstract class and is expected to be subclassed."""

    @abc.abstractmethod
    @property
    def location(self) -> str:
        """the location from where this launch description was loaded if applicable"""
        ...

    @abc.abstractmethod
    @property
    def method(self) -> str:
        """The method by which the launch description was generated"""
        ...

    @abc.abstractmethod
    def get_launch_description(self, context: LaunchContext) -> LaunchDescription:
        """Get the LaunchDescription, loading it if necessary."""
        ...

    @abc.abstractmethod
    def try_get_launch_description_without_context(self) -> Optional[LaunchDescription]:
        """
        Attempt to load the LaunchDescription without a context, return None if unsuccessful.

        This method is useful for trying to introspect the included launch
        description without visiting the user of this source.
        """
        ...
