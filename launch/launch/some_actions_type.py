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

"""
Module for SomeActionsType type.

.. deprecated:: 1.4.2
   Replaced by the more aptly named 'SomeEntitiesType'
"""

import warnings

from .some_entities_type import SomeEntitiesType, SomeEntitiesType_types_tuple

warnings.warn(
    "The 'SomeActionsType' type is deprecated. Use 'SomeEntitiesType' in your type"
    ' annotations instead!',
    UserWarning,
)

SomeActionsType = SomeEntitiesType
SomeActionsType_types_tuple = SomeEntitiesType_types_tuple
