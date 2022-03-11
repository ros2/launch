# Copyright 2018 Open Source Robotics Foundation, Inc.
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

"""Module for the PythonExpression substitution."""

import collections.abc
import itertools
import math
from typing import Iterable
from typing import List
from typing import Text

from launch.substitutions.text_substitution import TextSubstitution

from ..frontend import expose_substitution
from ..launch_context import LaunchContext
from ..some_substitutions_type import SomeSubstitutionsType
from ..substitution import Substitution
from ..substitutions import LaunchConfiguration
from ..utilities import ensure_argument_type
from ..utilities import normalize_to_list_of_substitutions
from ..utilities import perform_substitutions


def is_quoted(exp) -> bool:
    return isinstance(exp, TextSubstitution) and (not exp.quote)


@expose_substitution('eval')
class PythonExpression(Substitution):
    """
    Substitution that can access contextual local variables.

    The expression may contain Substitutions, but must return something that can
    be converted to a string with `str()`.
    It also may contain math symbols and functions.
    """

    def __init__(self, expression: SomeSubstitutionsType) -> None:
        """Create a PythonExpression substitution."""
        super().__init__()

        ensure_argument_type(
            expression,
            (str, Substitution, collections.abc.Iterable),
            'expression',
            'PythonExpression')

        self.__expression = normalize_to_list_of_substitutions(expression)

    @classmethod
    def parse(cls, data: Iterable[SomeSubstitutionsType]):
        """Parse `PythonExpression` substitution."""
        if len(data) == 1:
            return cls, {'expression': data[0]}
        expression = normalize_to_list_of_substitutions(itertools.chain.from_iterable(data))
        return cls, {'expression': expression}

    @property
    def expression(self) -> List[Substitution]:
        """Getter for expression."""
        return self.__expression

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return 'PythonExpr({})'.format(' + '.join([sub.describe() for sub in self.expression]))

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution by evaluating the expression."""
        if len(self.expression) != 1:
            expressions = [
                perform_substitutions(context, normalize_to_list_of_substitutions(exp))
                for exp in self.expression
            ]
            if '==' in expressions or '!=' in expressions:
                left = expressions[0]
                if is_quoted(self.expression[0]):
                    left = LaunchConfiguration(left, default=left).perform(context)

                right = expressions[2]
                if is_quoted(self.expression[2]):
                    right = LaunchConfiguration(right, default=right).perform(context)

                expression = f"'{left}' {expressions[1]} '{right}'"
                return str(eval(expression, {}, math.__dict__))

        return str(eval(perform_substitutions(context, self.expression), {}, math.__dict__))
