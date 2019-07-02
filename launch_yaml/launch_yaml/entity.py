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
from typing import Type
from typing import Union

from launch.launch_frontend import Entity as BaseEntity
from launch.launch_frontend.type_utils import check_type
from launch.launch_frontend.type_utils import SomeAllowedTypes


class Entity(BaseEntity):
    """Single item in the intermediate YAML front_end representation."""

    def __init__(
        self,
        element: dict,
        type_name: Text = None,
        *,
        parent: 'Entity' = None
    ) -> Text:
        """Constructor."""
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
        if not type(self.__element) in (dict, list):
            raise TypeError('Expected a dict or list, got {}'.format(type(self.element)))
        if isinstance(self.__element, dict):
            if 'children' not in self.__element:
                raise KeyError('Missing `children` key in Entity {}'.format(self.__type_name))
            children = self.__element['children']
        else:
            children = self.__element
        if not isinstance(children, list):
            raise TypeError('children should be a list')
        entities = []
        for child in children:
            if len(child) != 1:
                raise RuntimeError('Expected one root per child')
            type_name = list(child.keys())[0]
            entities.append(Entity(child[type_name], type_name))
        return entities

    def get_attr(
        self,
        name: Text,
        *,
        types:
            Optional[
                Union[
                    SomeAllowedTypes,
                    Type[List[BaseEntity]],
                    Type[List['Entity']],
                ]
            ] = str,
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
        if issubclass(types, List) and issubclass(types.__args__[0], BaseEntity):
            if isinstance(data, list) and isinstance(data[0], dict):
                return [Entity(child, name) for child in data]
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
