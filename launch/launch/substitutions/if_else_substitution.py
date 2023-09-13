# Copyright 2023 Open Source Robotics Foundation, Inc.
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

"""Module for the IfElseSubstitution substitution."""

from typing import List
from typing import Sequence
from typing import Text

from .substitution_failure import SubstitutionFailure
from ..frontend import expose_substitution
from ..launch_context import LaunchContext
from ..some_substitutions_type import SomeSubstitutionsType
from ..substitution import Substitution
from ..utilities import normalize_to_list_of_substitutions
from ..utilities import perform_substitutions
from ..utilities.type_utils import perform_typed_substitution


@expose_substitution('if')
class IfElseSubstitution(Substitution):
    """
    Substitution that conditionally returns one of two substitutions.

    Depending on whether the condition substitution evaluates to true, either it returns
    the if_value substitution or the else_value substitution.

    Example with a boolean launch configuration:

        .. doctest::

            >>> from launch.substitutions import LaunchConfiguration
            >>> subst = IfElseSubstitution(
            ...     LaunchConfiguration("arg"),
            ...     if_value="arg_evaluated_to_true",
            ...     else_value="arg_evaluated_to_false")

    Combine with boolean substitutions to create more complex conditions.
    Example with multiple boolean launch configurations:

        .. doctest::

            >>> from launch.substitutions import AllSubstitution
            >>> from launch.substitutions import EqualsSubstitution
            >>> from launch.substitutions import LaunchConfiguration
            >>> from launch.substitutions import NotSubstitution
            >>> subst = IfElseSubstitution(
            ...     AllSubstitution(EqualsSubstitution(LaunchConfiguration("arg1"),
            ...                                        LaunchConfiguration("arg2")),
            ...                     NotSubstitution(LaunchConfiguration("arg3"))),
            ...     if_value="all_args_evaluated_to_true",
            ...     else_value="at_least_one_arg_evaluated_to_false")

    """

    def __init__(self, condition: SomeSubstitutionsType,
                 if_value: SomeSubstitutionsType = '',
                 else_value: SomeSubstitutionsType = '') -> None:
        """Create a IfElseSubstitution substitution."""
        super().__init__()
        if if_value == else_value == '':
            raise RuntimeError('One of if_value and else_value must be specified')
        self._condition = normalize_to_list_of_substitutions(condition)
        self._if_value = normalize_to_list_of_substitutions(if_value)
        self._else_value = normalize_to_list_of_substitutions(else_value)

    @classmethod
    def parse(cls, data: Sequence[SomeSubstitutionsType]):
        """Parse `IfElseSubstitution` substitution."""
        if len(data) < 2 or len(data) > 3:
            raise TypeError('if substitution expects from 2 or 3 arguments')
        kwargs = {'condition': data[0], 'if_value': data[1]}
        if len(data) == 3:
            kwargs['else_value'] = data[2]
        return cls, kwargs

    @property
    def condition(self) -> List[Substitution]:
        """Getter for condition."""
        return self._condition

    @property
    def if_value(self) -> List[Substitution]:
        """Getter for if value."""
        return self._if_value

    @property
    def else_value(self) -> List[Substitution]:
        """Getter for else value."""
        return self._else_value

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return f'IfElseSubstitution({self.condition}, {self.if_value}, {self.else_value})'

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution by evaluating the condition."""
        try:
            condition = perform_typed_substitution(context, self.condition, bool)
        except (TypeError, ValueError) as e:
            raise SubstitutionFailure(e)

        if condition:
            return perform_substitutions(context, self.if_value)
        else:
            return perform_substitutions(context, self.else_value)
