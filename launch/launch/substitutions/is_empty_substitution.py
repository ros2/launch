# Copyright 2026 Open Source Robotics Foundation, Inc.
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

"""Module for the IsEmptySubstitution substitution."""

from typing import Any
from typing import Dict
from typing import List
from typing import Sequence
from typing import Text
from typing import Tuple
from typing import Type

from ..frontend.expose import expose_substitution
from ..launch_context import LaunchContext
from ..some_substitutions_type import SomeSubstitutionsType
from ..substitution import Substitution
from ..utilities import normalize_to_list_of_substitutions
from ..utilities import perform_substitutions


@expose_substitution('is-empty')
class IsEmptySubstitution(Substitution):
    """
    Substitution that checks whether a string is empty.

    Returns 'true' or 'false' strings depending on whether the input is empty.

    For example, checking if a launch configuration is empty:

    .. code-block:: python

        is_empty = IsEmptySubstitution(LaunchConfiguration('arg'))

    .. code-block:: xml

        <let name="empty_check" value="$(is-empty $(var arg))"/>

    .. code-block:: yaml

        - let:
            name: empty_check
            value: "$(is-empty $(var arg))"

    This can be useful for conditional logic based on whether a string value
    has content or not.
    """

    def __init__(self, value: SomeSubstitutionsType) -> None:
        """
        Create an IsEmptySubstitution.

        :param value: string or substitutions whose emptiness is checked
        """
        super().__init__()
        self.__value = normalize_to_list_of_substitutions(value)

    @classmethod
    def parse(
        cls, data: Sequence[SomeSubstitutionsType]
    ) -> Tuple[Type['IsEmptySubstitution'], Dict[str, Any]]:
        """Parse `IsEmptySubstitution` substitution."""
        if len(data) != 1:
            raise TypeError('is-empty substitution expects 1 argument')
        return cls, {'value': data[0]}

    @property
    def value(self) -> List[Substitution]:
        """Getter for the value to check."""
        return self.__value

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return 'IsEmpty({})'.format(
            ' + '.join([sub.describe() for sub in self.value]))

    def perform(self, context: LaunchContext) -> Text:
        """Perform substitutions and check if the result is empty."""
        result = perform_substitutions(context, self.value)
        return str(result == '').lower()
