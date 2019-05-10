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

"""Test parsing list attributes."""

from pathlib import Path
import xml.etree.ElementTree as ET

from launch_xml import Entity


def test_list():
    """Parse tags with list attributes."""
    tree = ET.parse(str(Path(__file__).parent / 'list.xml'))
    root = tree.getroot()
    root_entity = Entity(root)
    tags = root_entity.children
    assert tags[0].get_attr('attr', types='list[str]') == ['1', '2', '3']
    assert tags[0].get_attr('attr', types='list[int]') == [1, 2, 3]
    assert tags[0].get_attr('attr', types='list[float]') == [1., 2., 3.]


if __name__ == '__main__':
    test_list()
