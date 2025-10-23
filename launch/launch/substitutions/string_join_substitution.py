# Copyright 2025 Open Source Robotics Foundation, Inc.
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

from typing import Any, Dict, Iterable, List, Sequence, Text, Tuple, Type

from ..frontend.expose import expose_substitution
from ..launch_context import LaunchContext
from ..some_substitutions_type import SomeSubstitutionsType
from ..substitution import Substitution
from ..utilities import perform_substitutions


@expose_substitution('string-join')
class StringJoinSubstitution(Substitution):
    """
    Substitution that joins strings and/or other substitutions.

    This takes in a list of string components as substitutions.
    The substitutions for each string component are performed and concatenated,
    and then all string components are joined with a specified delimiter as seperation.

    For example:

    .. code-block:: python

        subdomain = LaunchConfiguration(variable_name='subdomain', default='docs')
        url = StringJoinSubstitution(
            [['https', '://', subdomain], 'ros', 'org'],
            delimiter='.'
        )

    .. code-block:: xml

        <launch>
            <arg name="subdomain" default="docs"/>
            <let name="url" value="$(string-join . https://$(var subdomain) ros org)"/>
        </launch>

    .. code-block:: yaml

            launch:
                - arg:
                    name: subdomain
                    default: "docs"
                - let:
                    name: url
                    value: "$(string-join . https://$(var subdomain) ros org)"

    If the ``subdomain`` launch configuration was set to ``docs`` and the ``delimiter`` to ``.``,
    then any of the above launch descriptions would result in a string equal to

    .. code-block:: python

        'https://docs.ros.org'
    """

    def __init__(
        self,
        substitutions: Iterable[SomeSubstitutionsType],
        delimiter: SomeSubstitutionsType = '',
    ) -> None:
        """
        Create a StringJoinSubstitution.

        :param substitutions: the list of string component substitutions to join
        :param delimiter: the text inbetween two consecutive components (default no text)
        """
        from ..utilities import normalize_to_list_of_substitutions

        self.__substitutions = [
            normalize_to_list_of_substitutions(string_component_substitutions)
            for string_component_substitutions in substitutions
        ]
        self.__delimiter = normalize_to_list_of_substitutions(delimiter)

    @property
    def substitutions(self) -> List[List[Substitution]]:
        """Getter for substitutions."""
        return self.__substitutions

    @property
    def delimiter(self) -> List[Substitution]:
        """Getter for delimiter."""
        return self.__delimiter

    @classmethod
    def parse(
        cls, data: Sequence[SomeSubstitutionsType]
    ) -> Tuple[Type['StringJoinSubstitution'], Dict[str, Any]]:
        """Parse `StringJoinSubstitution` substitution."""
        if len(data) < 2:
            raise TypeError(
                'string-join substitution expects at least 2 arguments: '
                '1 delimiter + at least 1 component'
            )
        return cls, {'delimiter': data[0], 'substitutions': data[1:]}

    def __repr__(self) -> Text:
        """Return a description of this substitution as a string."""
        string_components = [
            ' + '.join([s.describe() for s in component_substitutions])
            for component_substitutions in self.substitutions
        ]
        delimiter_component = ' + '.join([d.describe() for d in self.delimiter])
        return (
            f'StringJoinSubstitution(['
            f'{", ".join(string_components)}], delimiter={delimiter_component})'
        )

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution by retrieving the local variable."""
        string_components = [
            perform_substitutions(context, component_substitutions)
            for component_substitutions in self.substitutions
        ]
        delimiter_component = perform_substitutions(context, self.delimiter)
        return delimiter_component.join(string_components)
