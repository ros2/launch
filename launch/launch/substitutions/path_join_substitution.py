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

"""Module for the PathJoinSubstitution substitution."""

import os
from typing import Iterable
from typing import List
from typing import Text

from ..launch_context import LaunchContext
from ..some_substitutions_type import SomeSubstitutionsType
from ..substitution import Substitution
from ..utilities import normalize_to_list_of_substitutions
from ..utilities import perform_substitutions


class PathJoinSubstitution(Substitution):
    """
    Substitution that join paths, in a platform independent way.

    This takes in a list of path components as substitutions. The substitutions for each path
    component are performed and concatenated, and then all path components are joined.

    For example:

    .. code-block:: python

        PathJoinSubstitution([
            EnvironmentVariable('SOME_DIR'),
            'cfg',
            ['config_', LaunchConfiguration('map'), '.yml']
        ])

    Or:

    .. code-block:: python

        cfg_dir = PathJoinSubstitution([EnvironmentVariable('SOME_DIR'), 'cfg'])
        cfg_file = cfg_dir / ['config_', LaunchConfiguration('map'), '.yml']

    If the ``SOME_DIR`` environment variable was set to ``/home/user/dir`` and the ``map`` launch
    configuration was set to ``my_map``, this would result in a path equal equivalent to (depending
    on the platform):

    .. code-block:: python

        '/home/user/dir/cfg/config_my_map.yml'
    """

    def __init__(self, substitutions: Iterable[SomeSubstitutionsType]) -> None:
        """
        Create a PathJoinSubstitution.

        :param substitutions: the list of path component substitutions to join
        """
        from ..utilities import normalize_to_list_of_substitutions
        self.__substitutions = [
            normalize_to_list_of_substitutions(path_component_substitutions)
            for path_component_substitutions in substitutions
        ]

    @property
    def substitutions(self) -> List[List[Substitution]]:
        """Getter for variable_name."""
        return self.__substitutions

    def __repr__(self) -> Text:
        """Return a description of this substitution as a string."""
        path_components = [
            ' + '.join([s.describe() for s in component_substitutions])
            for component_substitutions in self.substitutions
        ]
        return f"PathJoinSubstitution('{', '.join(path_components)}')"

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitutions and join into a path."""
        path_components = [
            perform_substitutions(context, component_substitutions)
            for component_substitutions in self.substitutions
        ]
        return os.path.join(*path_components)

    def __truediv__(self, additional_path: SomeSubstitutionsType) -> 'PathJoinSubstitution':
        """Join path substitutions using the / operator, mimicking pathlib.Path operation."""
        return PathJoinSubstitution(
            self.substitutions + [normalize_to_list_of_substitutions(additional_path)])


class PathSubstitution(PathJoinSubstitution):
    """
    Thin wrapper on PathJoinSubstitution for more pathlib.Path-like construction.

    .. code-block:: python

        PathSubstitution(LaunchConfiguration('base_dir')) / 'sub_dir' / 'file_name'

    Which, for ``base_dir:=/my_dir``, results in (depending on the platform):

    .. code-block:: python

        /my_dir/sub_dir/file_name

    """

    def __init__(self, path: SomeSubstitutionsType):
        """
        Create a PathSubstitution.

        :param path: May be a single text or Substitution element,
        or an Iterable of them which are then joined
        """
        super().__init__(normalize_to_list_of_substitutions(path))
