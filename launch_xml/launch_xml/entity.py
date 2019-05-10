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

from typing import List
from typing import Optional
from typing import Text
from typing import Tuple
from typing import Union
import xml.etree.ElementTree as ET

from launch_frontend import Entity as BaseEntity
from launch_frontend.type_utils import get_typed_value


class Entity(BaseEntity):
    """Single item in the intermediate XML front_end representation."""

    def __init__(
        self,
        xml_element: ET.Element = None,
        *,
        parent: 'Entity' = None
    ) -> Text:
        """Construnctor."""
        self.__xml_element = xml_element
        self.__parent = parent

    @property
    def type_name(self) -> Text:
        """Get Entity type."""
        return self.__xml_element.tag

    @property
    def parent(self) -> Optional['Entity']:
        """Get Entity parent."""
        return self.__parent

    @property
    def children(self) -> List['Entity']:
        """Get the Entity's children."""
        return map(Entity, self.__xml_element)

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
        if types == 'list[Entity]':
            return_list = filter(lambda x: x.tag == name, self.__xml_element)
            return map(Entity, return_list)
        value = None
        if name in self.__xml_element.attrib:
            name_sep = name + '-sep'
            if name_sep not in self.__xml_element.attrib:
                value = self.__xml_element.attrib[name]
            else:
                sep = self.__xml_element.attrib[name_sep]
                value = self.__xml_element.attrib[name].split(sep)
        if value is None:
            if not optional:
                raise AttributeError(
                    'Attribute {} of type {} not found in Entity {}'.format(
                        name, types, self.type_name
                    )
                )
            else:
                return None
        try:
            value = get_typed_value(value, types)
        except ValueError:
            raise TypeError(
                'Attribute {} of Entity {} expected to be of type {}.'
                '`{}` can not be converted to one of those types'.format(
                    name, self.type_name, types, value
                )
            )
        return value
