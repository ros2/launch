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

"""Module for Parser class and parsing methods."""

import io
from typing import Text
from typing import Union

import launch
from launch.utilities import is_a

from pkg_resources import iter_entry_points

from .entity import Entity
from .expose import action_parse_methods
from .parse_substitution import default_parse_substitution

interpolation_fuctions = {
    entry_point.name: entry_point.load()
    for entry_point in iter_entry_points('launch_frontend.interpolate_substitution')
}

extensions_loaded = False


class Parser:
    """
    Abstract class for parsing actions, substitutions and descriptions.

    Implementations of the parser class, should override the load method.
    They could also override the parse_substitution method, or not.
    load_parser_extensions, parse_action and parse_description are not suposed to be overrided.
    """

    extensions_loaded = False
    frontend_parsers = None

    @classmethod
    def load_parser_extensions(cls):
        """Load parser extension, in order to get all the exposed substitutions and actions."""
        if cls.extensions_loaded is False:
            for entry_point in iter_entry_points('launch_frontend.parser_extension'):
                entry_point.load()
            cls.extensions_loaded = True

    @classmethod
    def load_parser_implementations(cls):
        """Load all the available frontend entities."""
        if cls.frontend_parsers is None:
            cls.frontend_parsers = {
                entry_point.name: entry_point.load()
                for entry_point in iter_entry_points('launch_frontend.parser')
            }

    @classmethod
    def parse_action(cls, entity: Entity) -> launch.Action:
        """Parse an action, using its registered parsing method."""
        cls.load_parser_extensions()
        if entity.type_name not in action_parse_methods:
            raise RuntimeError('Unrecognized entity of the type: {}'.format(entity.type_name))
        return action_parse_methods[entity.type_name](entity, cls)

    @classmethod
    def parse_substitution(cls, value: Text) -> launch.SomeSubstitutionsType:
        """Parse a substitution."""
        return default_parse_substitution(value)

    @classmethod
    def parse_description(cls, entity: Entity) -> launch.LaunchDescription:
        """Parse a launch description."""
        if entity.type_name != 'launch':
            raise RuntimeError('Expected \'launch\' as root tag')
        actions = [cls.parse_action(child) for child in entity.children]
        return launch.LaunchDescription(actions)

    @classmethod
    def load(
        cls,
        file: Union[str, io.TextIOBase],
    ) -> (Entity, 'Parser'):
        """Return an entity loaded with a markup file."""
        cls.load_parser_implementations()
        if is_a(file, str):
            # This automatically recognizes 'file.xml' or 'file.launch.xml'
            # as a launch file using the xml frontend.
            frontend_name = file.rsplit('.', 1)[1]
            if frontend_name in cls.frontend_parsers:
                return cls.frontend_parsers[frontend_name].load(file)
        # If not, apply brute force.
        # TODO(ivanpauno): Maybe, we want to force correct file naming.
        # In that case, we should raise an error here.
        # TODO(ivanpauno): Recognize a wrong formatted file error from
        # unknown front-end implementation error.
        for implementation in cls.frontend_parsers.values():
            try:
                return implementation.load(file)
            except Exception:
                pass
        raise RuntimeError('Not recognized front-end implementation.')
