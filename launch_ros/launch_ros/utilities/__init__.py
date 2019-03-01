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

"""
Module for descriptions of launchable entities.

Descriptions are not executable and are immutable so they can be reused by launch entities.
"""

from .evaluate_parameters import evaluate_parameters
from .normalize_parameters import normalize_parameters
from .normalize_remap_rule import normalize_remap_rule
from .normalize_remap_rule import normalize_remap_rules


__all__ = [
    'evaluate_parameters',
    'normalize_parameters',
    'normalize_remap_rule',
    'normalize_remap_rules',
]
