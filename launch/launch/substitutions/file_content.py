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

"""Module for the FileContent substitution."""

from typing import List
from typing import Sequence
from typing import Text

from .substitution_failure import SubstitutionFailure
from ..frontend import expose_substitution
from ..launch_context import LaunchContext
from ..some_substitutions_type import SomeSubstitutionsType
from ..substitution import Substitution


@expose_substitution('file-content')
class FileContent(Substitution):
    """
    Substitution that reads the contents of a file.

    If the file is not found a `SubstitutionFailure` error is raised.
    """

    def __init__(self, path: SomeSubstitutionsType) -> None:
        """Create a FileContent substitution."""
        super().__init__()

        from ..utilities import normalize_to_list_of_substitutions
        self.__path = normalize_to_list_of_substitutions(path)

    @classmethod
    def parse(cls, data: Sequence[SomeSubstitutionsType]):
        """Parse `FileContent` substitution."""
        if not data or len(data) != 1:
            raise AttributeError('file content substitutions expect 1 argument')
        kwargs = {'path': data[0]}
        return cls, kwargs

    @property
    def path(self) -> List[Substitution]:
        """Getter for path."""
        return self.__path

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return 'FileContent({})'.format(
            ', '.join([sub.describe() for sub in self.path]))

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution by evaluating the expression."""
        from ..utilities import perform_substitutions
        path = perform_substitutions(context, self.path)
        try:
            with open(path, 'r') as f:
                return f.read()
        except FileNotFoundError:
            raise SubstitutionFailure('File not found: {}'.format(path))
