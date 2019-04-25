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

from typing import Optional
from typing import Text
from xml.etree.ElementTree import Element

import launch_frontend


class Entity(launch_frontend.Entity):
    """Single item in the intermediate XML front_end representation."""

    def __init__(self, xml_element: Element,
                 *, parent: 'Entity' = None) -> Text:
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
    def children(self):
        """Get Entity children."""
        return [Entity(child) for child in self.__xml_element]

    def __getattr__(self, name):
        """Abstraction of how to access the xml tree."""
        if name in self.__xml_element.attrib:
            name_sep = name + '-sep'
            if name_sep not in self.__xml_element.attrib:
                return self.__xml_element.attrib[name]
            else:
                sep = self.__xml_element.attrib[name_sep]
                return self.__xml_element.attrib[name].split(sep)
        return_list = filter(lambda x: x.tag == name,
                             self.__xml_element)
        return_list = [Entity(item) for item in return_list]
        if not return_list:
            raise AttributeError(
                'Can not find attribute {} in Entity {}'.format(
                    name, self.type_name))
        return return_list
