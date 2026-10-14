# Copyright 2025 Open Source Robotics Foundation, Inc.
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

"""Module for the FindLaunchfile substitution."""

from pathlib import Path
from typing import Any
from typing import Dict
from typing import List
from typing import Sequence
from typing import Text
from typing import Tuple
from typing import Type


from .substitution_failure import SubstitutionFailure
from ..frontend import expose_substitution
from ..launch_context import LaunchContext
from ..some_substitutions_type import SomeSubstitutionsType
from ..substitution import Substitution


def name_matches_launchfile(name: Text, file: Path) -> bool:
    from ..frontend import Parser  # avoid circular import
    valid_extensions = {'py', *Parser.get_available_extensions()}
    valid_suffixes = {'', '.launch', '_launch'}

    if not file.is_file():
        return False

    # Check if the file has a valid launch extension
    extension = file.suffix
    if extension.startswith('.'):
        extension = extension[1:]
    if extension not in valid_extensions:
        return False

    # Full filenames allowed, check for full match
    if name == file.name:
        return True

    remainder = file.stem.removeprefix(name)
    if remainder == file.stem:
        # The filename did not begin with the search name
        return False
    elif remainder in valid_suffixes:
        return True

    return False


def find_launchfile_in_path(name: Text, path: Path) -> List[Path]:
    return [file for file in path.iterdir() if name_matches_launchfile(name, file)]


@expose_substitution('find-launchfile')
class FindLaunchfile(Substitution):
    """
    Substitution that tries to locate a launchfile by stem name in a directory.

    :raise: SubstitutionFailure on invalid search directory
    :raise: SubstitutionFailure when no matching launchfiles found
    :raise: SubstitutionFailure when more than 1 matching files found
    """

    def __init__(self, *, name: SomeSubstitutionsType, path: SomeSubstitutionsType) -> None:
        """Create a FindLaunchfile substitution."""
        super().__init__()

        from ..utilities import normalize_to_list_of_substitutions  # import here to avoid loop
        self.__name = normalize_to_list_of_substitutions(name)
        self.__path = normalize_to_list_of_substitutions(path)

    @classmethod
    def parse(cls, data: Sequence[SomeSubstitutionsType]
              ) -> Tuple[Type['FindLaunchfile'], Dict[str, Any]]:
        """Parse `FindLaunchfile` substitution."""
        if len(data) != 2:
            raise AttributeError(
                f'find-launchfile substitution expects 2 argument, {len(data)} given')
        return cls, {'name': data[0], 'path': data[1]}

    @property
    def name(self) -> List[Substitution]:
        """Getter for name."""
        return self.__name

    @property
    def path(self) -> List[Substitution]:
        """Getter for path."""
        return self.__path

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        name = ' + '.join([sub.describe() for sub in self.name])
        path = ' + '.join([sub.describe() for sub in self.path])
        return f'FindLaunchfile(name={name}, path={path})'

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution by locating the executable on the PATH."""
        from ..utilities import perform_substitutions  # import here to avoid loop
        name = perform_substitutions(context, self.name)
        path = Path(perform_substitutions(context, self.path))
        if not path.is_dir():
            raise SubstitutionFailure(f"Path '{path}' is not a directory")

        results = find_launchfile_in_path(name, path)
        if len(results) == 0:
            raise SubstitutionFailure(
                f"No launchfile matching name '{name}' found in directory '{path}'")
        elif len(results) > 1:
            raise SubstitutionFailure(
                f"Multiple launchfiles matching name '{name}' "
                f"found in directory '{path}': {results}")
        return str(results[0])
