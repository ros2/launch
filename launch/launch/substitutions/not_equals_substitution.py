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

"""Module for the NotEqualsSubstitution substitution."""

from typing import Any
from typing import Iterable
from typing import Optional
from typing import Text
from typing import Union

from .equals_substitution import EqualsSubstitution
from ..frontend import expose_substitution
from ..launch_context import LaunchContext


@expose_substitution('not-equals')
class NotEqualsSubstitution(EqualsSubstitution):
    """
    Substitution that checks if two inputs are not equal.

    Returns 'true' or 'false' strings depending on the result.
    """

    def __init__(
        self,
        left: Optional[Union[Any, Iterable[Any]]],
        right: Optional[Union[Any, Iterable[Any]]]
    ) -> None:
        """Create a NotEqualsSubstitution substitution."""
        super().__init__(left, right)

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return f'NotEqualsSubstitution({self.left} {self.right})'

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution."""
        return str(not (super().perform(context) == 'true')).lower()
