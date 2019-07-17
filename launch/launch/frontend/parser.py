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
from typing import Any
from typing import Text
from typing import Tuple
from typing import Union

from pkg_resources import iter_entry_points

from .entity import Entity
from .expose import instantiate_action
from .parse_substitution import parse_substitution
from .parse_substitution import replace_escaped_characters
from ..action import Action
from ..some_substitutions_type import SomeSubstitutionsType
from ..utilities import is_a

interpolation_fuctions = {
    entry_point.name: entry_point.load()
    for entry_point in iter_entry_points('launch.frontend.interpolate_substitution_method')
}

if False:
    from ..launch_description import LaunchDescription  # noqa: F401


class Parser:
    """
    Abstract class for parsing launch actions, substitutions and descriptions.

    Implementations of the parser class, should override the load method.
    They could also override the parse_substitution method, or not.
    load_launch_extensions, parse_action and parse_description are not suposed to be overrided.
    """

    extensions_loaded = False
    frontend_parsers = None

    @classmethod
    def load_launch_extensions(cls):
        """Load launch extensions, in order to get all the exposed substitutions and actions."""
        if cls.extensions_loaded is False:
            for entry_point in iter_entry_points('launch.frontend.launch_extension'):
                entry_point.load()
            cls.extensions_loaded = True

    @classmethod
    def load_parser_implementations(cls):
        """Load all the available frontend entities."""
        if cls.frontend_parsers is None:
            cls.frontend_parsers = {
                entry_point.name: entry_point.load()
                for entry_point in iter_entry_points('launch.frontend.parser')
            }

    def parse_action(self, entity: Entity) -> (Action, Tuple[Any]):
        """Parse an action, using its registered parsing method."""
        self.load_launch_extensions()
        return instantiate_action(entity, self)

    def parse_substitution(self, value: Text) -> SomeSubstitutionsType:
        """Parse a substitution."""
        return parse_substitution(value)

    def escape_characters(self, value: Text) -> SomeSubstitutionsType:
        """Escape characters in strings."""
        return replace_escaped_characters(value)

    def parse_description(self, entity: Entity) -> 'LaunchDescription':
        """Parse a launch description."""
        # Avoid recursive import
        from ..launch_description import LaunchDescription  # noqa: F811
        if entity.type_name != 'launch':
            raise RuntimeError("Expected 'launch' as root tag")
        actions = [self.parse_action(child) for child in entity.children]
        return LaunchDescription(actions)

    @classmethod
    def load(
        cls,
        file: Union[str, io.TextIOBase],
    ) -> (Entity, 'Parser'):
        """Return an entity loaded with a markup file."""
        cls.load_parser_implementations()
        if is_a(file, str):
            # This automatically recognizes the launch frontend markup
            # from the extension.
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
                if is_a(file, io.TextIOBase):
                    file.seek(0)
        raise RuntimeError('Not recognized front-end implementation.')
