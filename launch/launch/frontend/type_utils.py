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

"""Extra type utils for launch frontend implementations."""

from typing import List
from typing import Type
from typing import Union

from .entity import Entity
from ..utilities.type_utils import AllowedTypesType
from ..utilities.type_utils import check_is_typing_list


def check_is_list_entity(data_type: Union[AllowedTypesType, Type[List[Entity]]]) -> bool:
    """Check if `data_type` is a `typing.List` with elements of `Entity` type or derived."""
    return check_is_typing_list(data_type) and \
        issubclass(data_type.__args__[0], Entity)  # type: ignore
