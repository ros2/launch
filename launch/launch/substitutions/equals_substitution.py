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

from typing import Any
from typing import Iterable
from typing import Optional
from typing import Text
from typing import Union

from ..frontend import expose_substitution
from ..launch_context import LaunchContext
from ..some_substitutions_type import is_some_substitutions_type
from ..some_substitutions_type import SomeSubstitutionsType
from ..substitution import Substitution
from ..utilities import normalize_to_list_of_substitutions
from ..utilities.type_utils import perform_substitutions


@expose_substitution('equals')
class EqualsSubstitution(Substitution):
    """Substitution that checks if two inputs are equal."""

    def __init__(
        self,
        left: Optional[Union[Any, Iterable[Any]]],
        right: Optional[Union[Any, Iterable[Any]]]
    ) -> None:
        """Create an EqualsSubstitution substitution."""
        super().__init__()

        if is_some_substitutions_type(left):
            self.__left = normalize_to_list_of_substitutions(left)
        else:
            self.__left = left

        if is_some_substitutions_type(right):
            self.__right = normalize_to_list_of_substitutions(right)
        else:
            self.__right = right

    @classmethod
    def parse(cls, data: Iterable[SomeSubstitutionsType]):
        """Parse `EqualsSubstitution` substitution."""
        if len(data) != 2:
            raise TypeError('and substitution expects 2 arguments')
        return cls, {'left': data[0], 'right': data[1]}

    @property
    def left(self) -> Substitution:
        """Getter for left."""
        return self.__left

    @property
    def right(self) -> Substitution:
        """Getter for right."""
        return self.__right

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return f'EqualsSubstitution({self.left} {self.right})'

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution."""
        if is_some_substitutions_type(self.left):
            left = perform_substitutions(context, self.left)
        else:
            left = self.left

        if is_some_substitutions_type(self.right):
            right = perform_substitutions(context, self.right)
        else:
            right = self.right

        return str(left == right).lower()
