# Copyright 2020 Open Source Robotics Foundation, Inc.
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

"""Module for the Command substitution."""

import shlex
import subprocess
from typing import Iterable
from typing import List
from typing import Text

from .substitution_failure import SubstitutionFailure
from ..frontend.expose import expose_substitution
from ..launch_context import LaunchContext
from ..some_substitutions_type import SomeSubstitutionsType
from ..substitution import Substitution


@expose_substitution('command')
class Command(Substitution):
    """
    Substitution that gets the output of a command as a string.

    If the command is not found, fails, or has `stderr` output,
    a `SubstitutionFailure` error is raised.
    """

    def __init__(
        self,
        command: SomeSubstitutionsType,
    ) -> None:
        """
        Construct a command substitution.

        :param command: command to be executed. The substitutions will be performed, and
            `shlex.split` is applied to the result to form the arguments.
        """
        super().__init__()

        from ..utilities import normalize_to_list_of_substitutions  # import here to avoid loop
        self.__command = normalize_to_list_of_substitutions(command)

    @classmethod
    def parse(cls, data: Iterable[SomeSubstitutionsType]):
        """Parse `Command` substitution."""
        if len(data) < 1 or len(data) > 2:
            raise ValueError('command substitution expects 1 argument')
        return cls, {'command': data[0]}

    @property
    def command(self) -> List[Substitution]:
        """Getter for command."""
        return self.__command

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return 'Command({})'.format(' + '.join([sub.describe() for sub in self.command]))

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution by running the command and capturing it's output."""
        from ..utilities import perform_substitutions  # import here to avoid loop
        command = perform_substitutions(context, self.command)
        command = shlex.split(command)
        try:
            result = subprocess.run(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True)
        except FileNotFoundError as ex:
            raise SubstitutionFailure(f'File not found: {str(ex)}')
        if result.returncode != 0:
            raise SubstitutionFailure('executed command failed')
        if result.stderr:
            raise SubstitutionFailure('executed command showed stderr output')
        return result.stdout
