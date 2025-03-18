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

"""Module for the StringJoinSubstitution substitution."""

from typing import Iterable, List, Text

from ..launch_context import LaunchContext
from ..some_substitutions_type import SomeSubstitutionsType
from ..substitution import Substitution
from ..utilities import perform_substitutions


class StringJoinSubstitution(Substitution):
    """
    Substitution that joins strings and/or other substitutions.

    This takes in a list of string components as substitutions.
    The substitutions for each string component are performed and concatenated,
    and then all string components are joined with a specified delimiter as seperation.

    For example:

    .. code-block:: python

        StringJoinSubstitution(
            ['https://', LaunchConfiguration('subdomain')], 'ros', 'org'],
            delimiter='.'
        )

    If the ``subdomain`` launch configuration was set to ``docs``
    and the ``delimiter`` to ``.``, this would result in a string equal to

    .. code-block:: python

        'https://docs.ros.org'
    """

    def __init__(
        self, substitutions: Iterable[SomeSubstitutionsType], delimiter: str = ''
    ) -> None:
        """
        Create a StringJointSubstitution.

        :param substitutions: the list of string component substitutions to join
        :param delimiter: the text inbetween two consecutive components (default no text)
        """
        from ..utilities import normalize_to_list_of_substitutions

        self.__substitutions = [
            normalize_to_list_of_substitutions(string_component_substitutions)
            for string_component_substitutions in substitutions
        ]
        self.__delimiter = delimiter

    @property
    def substitutions(self) -> List[List[Substitution]]:
        """Getter for substitutions."""
        return self.__substitutions

    @property
    def delimiter(self) -> Text:
        """Getter for delimiter."""
        return self.__delimiter

    def __repr__(self) -> Text:
        """Return a description of this substitution as a string."""
        string_components = [
            ' + '.join([s.describe() for s in component_substitutions])
            for component_substitutions in self.substitutions
        ]
        return (
            f'StringJoinSubstitution(['
            f'{", ".join(string_components)}], delimiter={self.delimiter})'
        )

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution by retrieving the local variable."""
        string_components = [
            perform_substitutions(context, component_substitutions)
            for component_substitutions in self.substitutions
        ]
        return self.delimiter.join(string_components)
