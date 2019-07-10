# Copyright 2019 Open Source Robotics Foundation, Inc.
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

"""Module for Entity class."""

from typing import Any
from typing import List
from typing import Optional
from typing import Text
from typing import Union


class Entity:
    """Single item in the intermediate front_end representation."""

    @property
    def type_name(self) -> Text:
        """Get Entity type."""
        raise NotImplementedError()

    @property
    def parent(self) -> Optional['Entity']:
        """Get Entity parent."""
        raise NotImplementedError()

    @property
    def children(self) -> List['Entity']:
        """Get the Entity's children."""
        raise NotImplementedError()

    def get_attr(
        self,
        name: Text,
        *,
        data_type: Any = str,
        optional: bool = False
    ) -> Optional[Union[
        List[Union[int, str, float, bool]],
        Union[int, str, float, bool],
        List['Entity']
    ]]:
        """
        Access an attribute of the entity.

        By default, it will try to return it as an string.
        `types` states the expected types of the attribute. Type coercion or type checking is
        applied depending on the particular frontend.

        The allowed types are:
            - a scalar type i.e. `str`, `int`, `float`, `bool`;
            - a uniform list i.e `List[str]`, `List[int]`, `List[float]`, `List[bool]`;
            - a non-uniform list of known scalar types e.g. `List[Union[int, str]]`;
            - a non-uniform list of any scalar type i.e. `list` or `List`;
            - a `Union` of any of the above;
            - `List[Entity]`, see below.

        `types = None` works in the same way as:
            `Union[int, float, bool, list, str]`

        `List[Entity]` allows accessing a list of subentities with an specific name.
        Check the documentation of each specific frontend implementation to see how `list`
        and `List[Entity]` look like.

        If `optional` is `True` and the attribute cannot be found, `None` will be returned
        instead of raising `AttributeError`.

        :param name: name of the attribute
        :param types: type of the attribute to be read. Default to 'str'
        :param optional: when `True`, it doesn't raise an error when the attribute is not found.
            It returns `None` instead. Defaults to `False`
        :raises `AttributeError`: Attribute not found. Only possible if `optional` is `False`
        :raises `TypeError`: Attribute found but it is not of the correct type.
            Only happens in frontend implementations that do type checking
        :raises `ValueError`: Attribute found but can't be coerced to one of the types.
            Only happens in frontend implementations that do type coercion
        """
        raise NotImplementedError()
