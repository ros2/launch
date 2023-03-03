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

"""Module for the LaunchLogDir substitution."""

from typing import Sequence
from typing import Text

from ..frontend.expose import expose_substitution
from ..launch_context import LaunchContext
from ..logging import launch_config as launch_logging_config
from ..some_substitutions_type import SomeSubstitutionsType
from ..substitution import Substitution


@expose_substitution('launch_log_dir')
@expose_substitution('log_dir')
class LaunchLogDir(Substitution):
    """Substitution that returns the absolute path to the current launch log directory."""

    def __init__(self) -> None:
        """Create a LaunchLogDir substitution."""
        super().__init__()

    @classmethod
    def parse(cls, data: Sequence[SomeSubstitutionsType]):
        """Parse `LaunchLogDir` substitution."""
        if len(data) != 0:
            raise TypeError("launch_log_dir/log_dir substitution doesn't expect arguments")
        return cls, {}

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return 'LaunchLogDir()'

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution by returning the path to the current launch log directory."""
        return launch_logging_config.log_dir
