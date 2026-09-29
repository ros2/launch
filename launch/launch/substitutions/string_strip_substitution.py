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

"""Module for the StringStripSubstitution substitution."""

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


@expose_substitution('string-strip')
class StringStripSubstitution(Substitution):
    """
    Substitution that removes leading and trailing whitespace from a string.

    For example, command output can be stripped before it is composed with
    other substitutions:

    .. code-block:: python

        hostname = StringStripSubstitution(Command('hostname'))

    .. code-block:: xml

        <let name="hostname" value="$(string-strip $(command 'hostname'))"/>

    .. code-block:: yaml

        - let:
            name: hostname
            value: "$(string-strip $(command 'hostname'))"

    This strips whitespace only; it does not otherwise escape or transform the
    value for use in another language or expression.
    """

    def __init__(self, value: SomeSubstitutionsType) -> None:
        """
        Create a StringStripSubstitution.

        :param value: string or substitutions whose leading and trailing
            whitespace is removed
        """
        super().__init__()
        self.__value = normalize_to_list_of_substitutions(value)

    @classmethod
    def parse(
        cls, data: Sequence[SomeSubstitutionsType]
    ) -> Tuple[Type['StringStripSubstitution'], Dict[str, Any]]:
        """Parse `StringStripSubstitution` substitution."""
        if len(data) != 1:
            raise TypeError('string-strip substitution expects 1 argument')
        return cls, {'value': data[0]}

    @property
    def value(self) -> List[Substitution]:
        """Getter for the value to strip."""
        return self.__value

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return 'StringStrip({})'.format(
            ' + '.join([sub.describe() for sub in self.value]))

    def perform(self, context: LaunchContext) -> Text:
        """Perform substitutions and remove leading and trailing whitespace."""
        return perform_substitutions(context, self.value).strip()
