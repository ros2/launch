# Copyright 2023 Toyota Motor Corporation, Inc.
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

"""Module for the normalize_to_list_of_entities() utility function."""

from typing import Iterable
from typing import List

from ..launch_description_entity import LaunchDescriptionEntity
from ..some_entities_type import SomeEntitiesType


def normalize_to_list_of_entities(entities: Iterable[SomeEntitiesType]) ->\
        List[LaunchDescriptionEntity]:
    """Return a list of Substitutions given a variety of starting inputs."""
    flattened: List[LaunchDescriptionEntity] = []
    for entity in entities:
        if isinstance(entity, LaunchDescriptionEntity):
            flattened.append(entity)
        else:
            flattened.extend(entity)
    return flattened
