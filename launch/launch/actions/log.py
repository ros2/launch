# Copyright 2025 Open Source Robotics Foundation, Inc.
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

"""Module for the Log action."""

import logging
import sys
from typing import Any
from typing import Dict
from typing import List
from typing import Tuple
from typing import Type
import warnings

import launch.logging


from ..action import Action
from ..frontend import Entity
from ..frontend import expose_action
from ..frontend import Parser  # noqa: F401
from ..launch_context import LaunchContext
from ..some_substitutions_type import SomeSubstitutionsType
from ..substitution import Substitution
from ..utilities import normalize_to_list_of_substitutions


class LogInterface(Action):

    def __init__(self, *, msg: SomeSubstitutionsType,
                 level: SomeSubstitutionsType, **kwargs):
        """Create a Log action."""
        super().__init__(**kwargs)

        self.__msg = normalize_to_list_of_substitutions(msg)
        self.__level = normalize_to_list_of_substitutions(level)
        self.__logger = launch.logging.get_logger('launch.user')

    @property
    def msg(self) -> List[Substitution]:
        """Getter for self.__msg."""
        return self.__msg

    @property
    def level(self) -> List[Substitution]:
        """Getter for self.__level."""
        return self.__level

    def execute(self, context: LaunchContext) -> None:
        """Execute the action."""
        level_sub = ''.join([context.perform_substitution(sub)
                             for sub in self.level]).upper()

        if sys.version_info >= (3, 11):
            log_levels = logging.getLevelNamesMapping()
        else:
            # TODO: Remove after Python 3.11+ is minimum support
            log_levels = {'NOTSET': logging.NOTSET,
                          'DEBUG': logging.DEBUG,
                          'INFO': logging.INFO,
                          'WARNING': logging.WARNING,
                          'ERROR': logging.ERROR,
                          'CRITICAL': logging.CRITICAL}

        if level_sub not in log_levels:
            raise KeyError(f"Invalid log level '{level_sub}', expected: {log_levels.keys()}")

        level_int = log_levels[level_sub]

        self.__logger.log(level_int,
                          ''.join([context.perform_substitution(sub) for sub in self.msg])
                          )
        return None


@expose_action('log')
class Log(LogInterface):
    """Action that logs a message when executed."""

    @classmethod
    def parse(
        cls,
        entity: Entity,
        parser: 'Parser'
    ) -> Tuple[Type['Log'], Dict[str, Any]]:
        """Parse `log` tag."""
        _, kwargs = super().parse(entity, parser)
        kwargs['msg'] = parser.parse_substitution(entity.get_attr('message'))

        # Check if still using old log action
        level = entity.get_attr('level', optional=True)
        # TODO: Remove optional level for Release after L-turtle release
        if level is None:
            warnings.warn(
                'The action log now expects a log level.'
                ' Either provide one or switch to using the log_info action',
                stacklevel=2)
            level = 'INFO'

        kwargs['level'] = parser.parse_substitution(level)
        return cls, kwargs


class SharedLogSpecificParse(Action):

    @classmethod
    def parse(
        cls,
        entity: Entity,
        parser: 'Parser'
    ) -> Tuple[Type['SharedLogSpecificParse'], Dict[str, Any]]:
        """Parse `log_*` tag."""
        _, kwargs = super().parse(entity, parser)
        kwargs['msg'] = parser.parse_substitution(entity.get_attr('message'))
        return cls, kwargs


@expose_action('log_info')
class LogInfo(SharedLogSpecificParse, LogInterface):
    """Action that logs a message with level INFO when executed."""

    def __init__(self, *, msg: SomeSubstitutionsType, **kwargs):
        """Create a LogInfo action."""
        super().__init__(msg=msg, level='INFO', **kwargs)


@expose_action('log_warning')
class LogWarning(SharedLogSpecificParse, LogInterface):
    """Action that logs a message with level WARNING when executed."""

    def __init__(self, *, msg: SomeSubstitutionsType, **kwargs):
        """Create a LogWarning action."""
        super().__init__(msg=msg, level='WARNING', **kwargs)


@expose_action('log_debug')
class LogDebug(SharedLogSpecificParse, LogInterface):
    """Action that logs a message with level DEBUG when executed."""

    def __init__(self, *, msg: SomeSubstitutionsType, **kwargs):
        """Create a LogDebug action."""
        super().__init__(msg=msg, level='DEBUG', **kwargs)


@expose_action('log_error')
class LogError(SharedLogSpecificParse, LogInterface):
    """Action that logs a message with level ERROR when executed."""

    def __init__(self, *, msg: SomeSubstitutionsType, **kwargs):
        """Create a LogError action."""
        super().__init__(msg=msg, level='ERROR', **kwargs)
