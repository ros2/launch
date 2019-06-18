# Copyright 2019 Open Source Robotics Foundation, Inc.
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

"""Module for the AnyLaunchDescriptionSource class."""

from launch_frontend import Parser

from .python_launch_file_utilities import get_launch_description_from_python_launch_file
from .python_launch_file_utilities import InvalidPythonLaunchFileError
from ..launch_description_source import LaunchDescriptionSource
from ..some_substitutions_type import SomeSubstitutionsType


class AnyLaunchDescriptionSource(LaunchDescriptionSource):
    """
    Encapsulation of a launch file, which can be loaded during launch.

    This try to load the file as a python launch file, or as a frontend launch file.
    It is recommended to use a specific `LaunchDescriptionSource` subclasses when possible.
    """

    def __init__(
        self,
        launch_file_path: SomeSubstitutionsType,
    ) -> None:
        """
        Constructor.

        The given file path should be to a launch file of any style.
        The path should probably be absolute, since the current working
        directory will be wherever the launch file was run from, which might
        change depending on the situation.
        The path can be made up of Substitution instances which are expanded
        when :py:meth:`get_launch_description()` is called.

        :param launch_file_path: the path to the launch file
        """
        super().__init__(
            None,
            launch_file_path,
            'interpreted launch file',
        )

    def _get_launch_description(self, location):
        """Get the LaunchDescription from location."""
        launch_description = None
        try:
            launch_description = get_launch_description_from_python_launch_file(location)
        except InvalidPythonLaunchFileError:
            pass
        try:
            root_entity, parser = Parser.load(location)
            launch_description = parser.parse_description(root_entity)
        except RuntimeError:
            pass
        if launch_description is None:
            raise RuntimeError('Can not load launch file')
        return launch_description
