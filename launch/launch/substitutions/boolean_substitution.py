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

"""Module for boolean substitutions."""

from typing import Iterable
from typing import Text

from .substitution_failure import SubstitutionFailure
from ..frontend import expose_substitution
from ..launch_context import LaunchContext
from ..some_substitutions_type import SomeSubstitutionsType
from ..substitution import Substitution
from ..utilities import normalize_to_list_of_substitutions
from ..utilities.type_utils import perform_typed_substitution


@expose_substitution('not')
class NotSubstitution(Substitution):
    """Substitution that returns 'not' of the input boolean value."""

    def __init__(self, value: SomeSubstitutionsType) -> None:
        """Create a NotSubstitution substitution."""
        super().__init__()

        self.__value = normalize_to_list_of_substitutions(value)

    @classmethod
    def parse(cls, data: Iterable[SomeSubstitutionsType]):
        """Parse `NotSubstitution` substitution."""
        if len(data) != 1:
            raise TypeError('not substitution expects 1 argument')
        return cls, {'value': data[0]}

    @property
    def value(self) -> Substitution:
        """Getter for value."""
        return self.__value

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return f'NotSubstitution({self.value})'

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution."""
        try:
            condition = perform_typed_substitution(context, self.value, bool)
        except (TypeError, ValueError) as e:
            raise SubstitutionFailure(e)

        return str(not condition).lower()


@expose_substitution('and')
class AndSubstitution(Substitution):
    """Substitution that returns 'and' of the input boolean values."""

    def __init__(self, left: SomeSubstitutionsType, right: SomeSubstitutionsType) -> None:
        """Create a AndSubstitution substitution."""
        super().__init__()

        self.__left = normalize_to_list_of_substitutions(left)
        self.__right = normalize_to_list_of_substitutions(right)

    @classmethod
    def parse(cls, data: Iterable[SomeSubstitutionsType]):
        """Parse `AndSubstitution` substitution."""
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
        return f'AndSubstitution({self.left} {self.right})'

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution."""
        try:
            left_condition = perform_typed_substitution(context, self.left, bool)
        except (TypeError, ValueError) as e:
            raise SubstitutionFailure(e)
        try:
            right_condition = perform_typed_substitution(context, self.right, bool)
        except (TypeError, ValueError) as e:
            raise SubstitutionFailure(e)

        return str(left_condition and right_condition).lower()


@expose_substitution('or')
class OrSubstitution(Substitution):
    """Substitution that returns 'or' of the input boolean values."""

    def __init__(self, left: SomeSubstitutionsType, right: SomeSubstitutionsType) -> None:
        """Create a AndSubstitution substitution."""
        super().__init__()

        self.__left = normalize_to_list_of_substitutions(left)
        self.__right = normalize_to_list_of_substitutions(right)

    @classmethod
    def parse(cls, data: Iterable[SomeSubstitutionsType]):
        """Parse `AndSubstitution` substitution."""
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
        return f'AndSubstitution({self.left} {self.right})'

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution."""
        try:
            left_condition = perform_typed_substitution(context, self.left, bool)
        except (TypeError, ValueError) as e:
            raise SubstitutionFailure(e)
        try:
            right_condition = perform_typed_substitution(context, self.right, bool)
        except (TypeError, ValueError) as e:
            raise SubstitutionFailure(e)

        return str(left_condition or right_condition).lower()
