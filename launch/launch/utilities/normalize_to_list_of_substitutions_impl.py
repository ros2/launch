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

"""Module for the normalize_to_list_of_substitutions() utility function."""

from pathlib import Path
from typing import Iterable
from typing import List
from typing import Union

from .class_tools_impl import is_a_subclass
from ..some_substitutions_type import SomeSubstitutionsType
from ..substitution import Substitution


def normalize_to_list_of_substitutions(subs: SomeSubstitutionsType) -> List[Substitution]:
    """Return a list of Substitutions given a variety of starting inputs."""
    # Avoid recursive import
    from ..substitutions import TextSubstitution, PathSubstitution

    def normalize(x: Union[str, Substitution, Path]) -> Substitution:
        if isinstance(x, Substitution):
            return x
        if isinstance(x, str):
            return TextSubstitution(text=x)
        if isinstance(x, Path):
            return PathSubstitution(path=x)
        raise TypeError(
            "Failed to normalize given item of type '{}', when only "
            "'str' or 'launch.Substitution' were expected.".format(type(x)))

    if isinstance(subs, str):
        return [TextSubstitution(text=subs)]
    elif isinstance(subs, Path):
        return [PathSubstitution(path=subs)]
    elif is_a_subclass(subs, Substitution):
        return [subs]
    elif isinstance(subs, Iterable):
        return [normalize(y) for y in subs]

    raise TypeError(f'{subs} is not a valid SomeSubstitutionsType.')
