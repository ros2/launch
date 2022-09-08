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

"""Module for SomeSubstitutionsType type."""

import collections.abc
from typing import Any
from typing import Iterable
from typing import Optional
from typing import Text
from typing import Union

from .substitution import Substitution

SomeSubstitutionsType = Union[
    Text,
    Substitution,
    Iterable[Union[Text, Substitution]],
]

SomeSubstitutionsType_types_tuple = (
    str,
    Substitution,
    collections.abc.Iterable,
)


def is_some_substitutions_type(input: Optional[Union[Any, Iterable[Any]]]) -> bool:
    """
    Check if input is instance of members of SomeSubstitutionsType.

    It is not possible to do a direct comparison, and typing.get_args() doesn't seem to work, so
    we're forced to use this very inelegant solution...
    """
    if isinstance(input, Iterable):
        return all(
            (isinstance(i, Substitution) or isinstance(i, Text)) for i in input
        )
    else:
        return any([
            isinstance(input, Substitution),
            isinstance(input, Text)
        ])
