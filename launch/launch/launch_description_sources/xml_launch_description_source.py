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

"""Module for the XMLLaunchDescriptionSource class."""

from launch_xml import Parser

from . import FrontendLaunchDescriptionSource
from ..some_substitutions_type import SomeSubstitutionsType


class XMLLaunchDescriptionSource(FrontendLaunchDescriptionSource):
    """Encapsulation of a XML launch file, which can be loaded during launch."""

    def __init__(
        self,
        launch_file_path: SomeSubstitutionsType,
    ) -> None:
        """
        Constructor.

        The given file path should be to a launch XML style file (`.launch.xml`).
        The path should probably be absolute, since the current working
        directory will be wherever the launch file was run from, which might
        change depending on the situation.
        The path can be made up of Substitution instances which are expanded
        when :py:meth:`get_launch_description()` is called.

        :param launch_file_path: the path to the launch file
        """
        super().__init__(
            launch_file_path,
            method='interpreted XML launch file',
            parser=Parser
        )
