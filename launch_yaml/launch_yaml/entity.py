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

"""Module for YAML Entity class."""

from typing import List
from typing import Optional
from typing import Text
from typing import Tuple
from typing import Union

from launch_frontend import Entity as BaseEntity
from launch_frontend.type_utils import check_type


class Entity(BaseEntity):
    """Single item in the intermediate YAML front_end representation."""

    def __init__(
        self,
        element: dict,
        *,
        type_name: Optional[Text] = None,
        parent: 'Entity' = None
    ) -> Text:
        """Construnctor."""
        if type_name is None:
            if len(element) != 1:
                raise RuntimeError('Expected a len 1 dictionary')
            self.__type_name = list(element.keys())[0]
            self.__element = element[self.__type_name]
        else:
            self.__type_name = type_name
            self.__element = element
        self.__parent = parent

    @property
    def type_name(self) -> Text:
        """Get Entity type."""
        return self.__type_name

    @property
    def parent(self) -> Optional['Entity']:
        """Get Entity parent."""
        return self.__parent

    @property
    def children(self) -> List['Entity']:
        """Get the Entity's children."""
        if not isinstance(self.__element, list):
            raise TypeError('Expected a list, got {}'.format(type(self.element)))
        return [Entity(child) for child in self.__element]

    def get_attr(
        self,
        name: Text,
        *,
        types: Union[Text, Tuple[Text]] = 'str',
        optional: bool = False
    ) -> Optional[Union[
        Text,
        int,
        float,
        List[Text],
        List[int],
        List[float],
        List['Entity']
    ]]:
        """Access an attribute of the entity."""
        if name not in self.__element:
            if not optional:
                raise AttributeError(
                    'Can not find attribute {} in Entity {}'.format(
                        name, self.type_name))
            else:
                return None
        data = self.__element[name]
        if types == 'list[Entity]':
            if isinstance(data, list) and isinstance(data[0], dict):
                return [Entity(child, type_name=name) for child in data]
            raise TypeError(
                'Attribute {} of Entity {} expected to be a list of dictionaries.'.format(
                    name, self.type_name
                )
            )
        if not check_type(data, types):
            raise TypeError(
                'Attribute {} of Entity {} expected to be of type {}, got {}'.format(
                    name, self.type_name, types, type(data)
                )
            )
        return data
