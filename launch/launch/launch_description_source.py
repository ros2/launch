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

"""Module for the LaunchDescriptionSource class."""

from .launch_description import LaunchDescription


class LaunchDescriptionSource:
    """Encapsulation of a launch description, where it comes from, and how it was generated."""

    def __init__(
        self,
        launch_description: LaunchDescription,
        location: str = '<string>',
        method: str = 'unspecified mechanism from a script',
    ) -> None:
        """
        Constructor.

        For example, loading a file called ``example.launch.py`` for inclusion
        might end up setting `location` to ``/path/to/example.launch.py`` and
        the ``method`` to be ``interpreted python launch file``.

        :param launch_description: the launch description that this source represents
        :param location: the location from where this launch description was loaded if applicable
        :param method: the method by which the launch description was generated
        """
        self.__launch_description = launch_description
        self.__location = location
        self.__method = method

    @property
    def launch_description(self) -> LaunchDescription:
        """Getter for self.__launch_description."""
        return self.__launch_description

    @property
    def location(self) -> str:
        """Getter for self.__location."""
        return self.__location

    @property
    def method(self) -> str:
        """Getter for self.__method."""
        return self.__method
