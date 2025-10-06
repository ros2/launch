# Copyright 2022 Open Source Robotics Foundation, Inc.
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

"""Module for the EqualsSubstitution substitution."""

import math

from typing import Any
from typing import cast
from typing import Iterable
from typing import List
from typing import Optional
from typing import Sequence
from typing import Text
from typing import Union

from ..frontend import expose_substitution
from ..launch_context import LaunchContext
from ..some_substitutions_type import SomeSubstitutionsType
from ..substitution import Substitution
from ..utilities import normalize_to_list_of_substitutions
from ..utilities.type_utils import is_substitution, perform_substitutions


def _str_is_bool(input_str: Text) -> bool:
    """Check if string input is convertible to a boolean."""
    if not isinstance(input_str, Text):
        return False
    else:
        return input_str.lower() in ('true', 'false', '1', '0')


def _str_is_float(input_str: Text) -> bool:
    """Check if string input is convertible to a float."""
    try:
        float(input_str)
        return True
    except ValueError:
        return False


@expose_substitution('equals')
class EqualsSubstitution(Substitution):
    """
    Substitution that checks if two inputs are equal.

    Returns 'true' or 'false' strings depending on the result.
    """

    def __init__(
        self,
        left: Optional[Union[Any, Iterable[Any]]],
        right: Optional[Union[Any, Iterable[Any]]]
    ) -> None:
        """Create an EqualsSubstitution substitution."""
        super().__init__()

        if not is_substitution(left):
            if left is None:
                left = ''
            elif isinstance(left, bool):
                left = str(left).lower()
            else:
                left = str(left)

        if not is_substitution(right):
            if right is None:
                right = ''
            elif isinstance(right, bool):
                right = str(right).lower()
            else:
                right = str(right)

        # mypy is unable to understand that if we passed in the `else` branch
        # above, left & right must be substitutions. Unfortunately due to the
        # way is_substitution is written, it's hard to get mypy to typecheck
        # it correctly, so cast here.
        self.__left = normalize_to_list_of_substitutions(
                cast(Union[str, Substitution, Sequence[Union[str, Substitution]]], left)
                )
        self.__right = normalize_to_list_of_substitutions(
                cast(Union[str, Substitution, Sequence[Union[str, Substitution]]], right)
                )

    @classmethod
    def parse(cls, data: Sequence[SomeSubstitutionsType]):
        """Parse `EqualsSubstitution` substitution."""
        if len(data) != 2:
            raise TypeError('and substitution expects 2 arguments')
        return cls, {'left': data[0], 'right': data[1]}

    @property
    def left(self) -> List[Substitution]:
        """Getter for left."""
        return self.__left

    @property
    def right(self) -> List[Substitution]:
        """Getter for right."""
        return self.__right

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return f'EqualsSubstitution({self.left} {self.right})'

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution."""
        left = perform_substitutions(context, self.left)
        right = perform_substitutions(context, self.right)

        # Special case for booleans
        if _str_is_bool(left) and _str_is_bool(right):
            return str((left.lower() in ('true', '1')) == (right.lower() in ('true', '1'))).lower()

        # Special case for floats (epsilon closeness)
        if _str_is_float(left) and _str_is_float(right):
            return str(math.isclose(float(left), float(right))).lower()

        return str(left == right).lower()
