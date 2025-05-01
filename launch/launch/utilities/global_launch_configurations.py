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

from typing import Set
from typing import Text

from .normalize_to_list_of_substitutions_impl import normalize_to_list_of_substitutions
from .perform_substitutions_impl import perform_substitutions
from ..launch_context import LaunchContext
from ..some_substitutions_type import SomeSubstitutionsType


def register_global(context: LaunchContext, name: SomeSubstitutionsType) -> None:
    """
    Register a launch configuration to the globals set.

    This allows it to be accessible in when scoped including a launch description.
    """
    globals_set: Set[Text] = getattr(context.locals, 'globals', {'globals'})
    globals_set.add(perform_substitutions(context, normalize_to_list_of_substitutions(name)))
    context.extend_locals({'globals': globals_set})


def unregister_global(context: LaunchContext, name: SomeSubstitutionsType) -> None:
    """Unregisters a launch configuration to the globals set."""
    globals_set: Set[Text] = getattr(context.locals, 'globals', {'globals'})
    globals_set.discard(perform_substitutions(context, normalize_to_list_of_substitutions(name)))
    context.extend_locals({'globals': globals_set})
