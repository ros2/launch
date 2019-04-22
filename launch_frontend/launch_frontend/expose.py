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

"""Module which adds methods for exposing parsing methods."""

import inspect
from typing import Text

substitution_parse_methods = {}
action_parse_methods = {}


def __expose_impl(name: Text, parse_methods_map: dict, exposed_type: Text):
    """Implementats expose decorators."""
    def expose_impl_decorator(exposed):
        found_parse_method = None
        if inspect.isclass(exposed):
            if 'parse' in dir(exposed):
                found_parse_method = exposed.parse
        elif callable(exposed):
            found_parse_method = exposed
        if not found_parse_method:
            raise RuntimeError(
                'Exposed {} parser for {} is not a callable or a class'
                ' containg a parse method'.format(exposed_type, name)
            )
        if name in parse_methods_map and found_parse_method is not parse_methods_map[name]:
            raise RuntimeError(
                'Two parsing methods exposed with same name.'
            )
        parse_methods_map[name] = found_parse_method
        return exposed
    return expose_impl_decorator


def expose_substitution(name: Text):
    """Exposes a substitution."""
    return __expose_impl(name, substitution_parse_methods, 'substitution')


def expose_action(name: Text):
    """Exposes an action."""
    return __expose_impl(name, action_parse_methods, 'action')
