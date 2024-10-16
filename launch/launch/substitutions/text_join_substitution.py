# Copyright 2024 Open Source Robotics Foundation, Inc.
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

"""Module for the TextJoinSubstitution substitution."""

from typing import Iterable
from typing import Text
from typing import Union

from ..launch_context import LaunchContext
from ..substitution import Substitution


class TextJoinSubstitution(Substitution):
    """Substitution that join texts using a separator."""

    def __init__(
        self, substitutions: Iterable[Union[Text, Substitution]], separator: Text = ""
    ) -> None:
        """Create a TextJoinSubstitution."""
        from ..utilities import normalize_to_list_of_substitutions

        self.__substitutions = normalize_to_list_of_substitutions(substitutions)
        self.__separator = separator

    @property
    def substitutions(self) -> Iterable[Substitution]:
        """Getter for variable_name."""
        return self.__substitutions

    @property
    def separator(self) -> Text:
        """Getter for separator."""
        return self.__separator

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return "LocalVar('{}')".format(
            " + ".join([s.describe() for s in self.substitutions])
        )

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution by joining the texts."""
        performed_substitutions = [sub.perform(context) for sub in self.__substitutions]
        return self.__separator.join(performed_substitutions)
