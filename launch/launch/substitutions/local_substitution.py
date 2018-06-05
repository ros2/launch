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

"""Module for the LocalSubstitution substitution."""

from typing import Text

from ..launch_context import LaunchContext
from ..substitution import Substitution
from ..utilities import ensure_argument_type


class LocalSubstitution(Substitution):
    """Substitution that can access contextual local variables."""

    def __init__(self, *, expression: Text) -> None:
        """Constructor."""
        super().__init__()

        ensure_argument_type(expression, str, 'expression', 'LocalSubstitution')

        self.__expression = expression

    @property
    def expression(self) -> Text:
        """Getter for expression."""
        return self.__expression

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution by looking up the environment expression."""
        return eval('context.locals.' + self.expression)
