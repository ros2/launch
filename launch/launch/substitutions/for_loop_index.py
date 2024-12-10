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

"""Module for the ForLoopIndex substitution."""

from typing import List
from typing import Sequence
from typing import Text

from .local_substitution import LocalSubstitution
from ..frontend import expose_substitution
from ..launch_context import LaunchContext
from ..logging import get_logger
from ..some_substitutions_type import SomeSubstitutionsType
from ..substitution import Substitution
from ..utilities import perform_substitutions


@expose_substitution('index')
class ForLoopIndex(Substitution):
    """Substitution for a ForLoop iteration index value."""

    def __init__(
        self,
        name: SomeSubstitutionsType,
    ) -> None:
        """
        Create a ForLoopIndex.

        :param name: the name of the ForLoop index which this substitution is part of
        """
        super().__init__()

        from ..utilities import normalize_to_list_of_substitutions  # import here to avoid loop
        self._name = normalize_to_list_of_substitutions(name)
        self._logger = get_logger(__name__)

    @property
    def name(self) -> List[Substitution]:
        return self._name

    def describe(self) -> Text:
        return f"ForLoopIndex(name={' + '.join([sub.describe() for sub in self._name])})"

    @classmethod
    def parse(cls, data: Sequence[SomeSubstitutionsType]):
        if len(data) != 1:
            raise ValueError('ForLoopIndex substitution expects 1 argument')
        kwargs = {}
        kwargs['name'] = data[0]
        return cls, kwargs

    def perform(self, context: LaunchContext) -> Text:
        name = perform_substitutions(context, self._name)
        self._logger.debug('name=' + name)
        index_substitution = LocalSubstitution(name)
        index = perform_substitutions(context, [index_substitution])
        self._logger.debug('index=' + index)
        return index
