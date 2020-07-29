# Copyright 2019 Open Source Robotics Foundation, Inc.
# Copyright 2020 Open Avatar Inc.
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

import os.path
from typing import List
from typing import Optional
from typing import Text
from typing import TextIO
from typing import Type
from typing import TYPE_CHECKING
from typing import Union

from pkg_resources import iter_entry_points

from .entity import Entity
from .expose import instantiate_action
from .parse_substitution import parse_if_substitutions
from .parse_substitution import parse_substitution
from .parse_substitution import replace_escaped_characters
from ..action import Action
from ..invalid_launch_file_error import InvalidLaunchFileError
from ..substitution import Substitution
from ..utilities.type_utils import NormalizedValueType
from ..utilities.type_utils import StrSomeValueType
from ..utilities.typing_file_path import FilePath

if TYPE_CHECKING:
    from ..launch_description import LaunchDescription


class InvalidFrontendLaunchFileError(InvalidLaunchFileError):
    """Exception raised when the given frontend launch file is not valid."""

    pass


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

    def parse_action(self, entity: Entity) -> Action:
        """Parse an action, using its registered parsing method."""
        self.load_launch_extensions()
        return instantiate_action(entity, self)

    def parse_substitution(self, value: Text) -> List[Substitution]:
        """Parse a substitution."""
        return parse_substitution(value)

    def parse_if_substitutions(
        self, value: StrSomeValueType
    ) -> NormalizedValueType:
        """See :py:func:`launch.frontend.parser.parse_if_substitutions`."""
        return parse_if_substitutions(value)

    def escape_characters(self, value: Text) -> Text:
        """Escape characters in strings."""
        return replace_escaped_characters(value)

    def parse_description(self, entity: Entity) -> 'LaunchDescription':
        """Parse a launch description."""
        # Avoid recursive import
        from ..launch_description import LaunchDescription  # noqa: F811
        if entity.type_name != 'launch':
            raise RuntimeError("Expected 'launch' as root tag")
        deprecated = entity.get_attr('deprecated', optional=True)
        actions = [self.parse_action(child) for child in entity.children]
        return LaunchDescription(actions, deprecated_reason=deprecated)

    @classmethod
    def get_available_extensions(cls) -> List[Text]:
        """Return the registered extensions."""
        cls.load_parser_implementations()
        return cls.frontend_parsers.keys()

    @classmethod
    def is_extension_valid(
        cls,
        extension: Text,
    ) -> bool:
        """Return an entity loaded with a markup file."""
        cls.load_parser_implementations()
        return extension in cls.frontend_parsers

    @classmethod
    def get_parser_from_extension(
        cls,
        extension: Text,
    ) -> Optional[Type['Parser']]:
        """Return an entity loaded with a markup file."""
        cls.load_parser_implementations()
        try:
            return cls.frontend_parsers[extension]
        except KeyError:
            raise RuntimeError('Not recognized frontend implementation')

    @classmethod
    def load(
        cls,
        file: Union[FilePath, TextIO],
    ) -> (Entity, 'Parser'):
        """
        Parse an Entity from a markup language-based launch file.

        Parsers are exposed and provided by available frontend implementations.
        To choose the right parser, it'll first attempt to infer the launch
        description format based on the filename extension, if any.
        If format inference fails, it'll try all available parsers one after the other.
        """
        # Imported here, to avoid recursive import.
        cls.load_parser_implementations()

        try:
            fileobj = open(file, 'r')
            didopen = True
        except TypeError:
            fileobj = file
            didopen = False

        try:
            filename = getattr(fileobj, 'name', '')
            # file extension without leading '.'
            extension = os.path.splitext(filename)[1][1:]

            sorted_parsers = sorted(cls.frontend_parsers.items())
            implementations = [v for k, v in sorted_parsers if k == extension] + [
                v for k, v in sorted_parsers if k != extension
            ]

            exceptions = []
            for implementation in implementations:
                try:
                    return implementation.load(fileobj)
                except Exception as ex:
                    exceptions.append(ex)
                    fileobj.seek(0)
            extension = '' if not cls.is_extension_valid(extension) else extension
            raise InvalidFrontendLaunchFileError(extension, likely_errors=exceptions)
        finally:
            if didopen:
                fileobj.close()
