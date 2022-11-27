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
# POSSIBILITY OF SUCH DAMAGE.

"""Module for OrCondition class."""

from typing import List, Text, Union

from .evaluate_condition_expression_impl import evaluate_condition_expression
from ..condition import Condition
from ..launch_context import LaunchContext
from ..some_substitutions_type import SomeSubstitutionsType
from ..utilities import normalize_to_list_of_substitutions


class OrCondition(Condition):
    """
    Encapsulates an or condition to be evaluated when launching.

    This condition takes a list or tuple of string expressions that are lexically evaluated as
    booleans, but the expressions may consist of launch.Substitution instances.

    This condition will raise a TypeError if a list or tuple is not provided.
    """

    def __init__(
        self,
        predicate_expressions: List[Union[SomeSubstitutionsType, Condition]]
    ) -> None:
        if isinstance(predicate_expressions, str) or len(predicate_expressions) == 1:
            raise TypeError(
                'Only given a single expression. Two or more expressions are expected.'
            )
        self.__conditions = []
        to_be_normalized = []
        # Separate Conditions to be evaluated on their own
        for pe in predicate_expressions:
            if isinstance(pe, Condition):
                self.__conditions.append(pe)
            else:
                to_be_normalized.append(pe)
        self.__substitutions = [
            normalize_to_list_of_substitutions(condition) for condition in to_be_normalized
        ]
        super().__init__(predicate=self._predicate_func)

    def _predicate_func(self, context: LaunchContext) -> bool:
        return any(
            [evaluate_condition_expression(context, sub) for sub in self.__substitutions]
        ) or any([condition.evaluate(context) for condition in self.__conditions])

    def describe(self) -> Text:
        return self.__repr__()
