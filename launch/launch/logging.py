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

"""Module for the launch specific logging."""

import datetime

# Expose entire logging module.

from logging import DEBUG
from logging import ERROR
from logging import FATAL
from logging import FileHandler
from logging import Formatter
from logging import Handler
from logging import INFO
from logging import Logger
from logging import NOTSET
from logging import root
from logging import setLoggerClass
from logging import shutdown
from logging import StreamHandler
from logging import WARN

import logging.handlers as handlers

import os
import socket
import sys


__all__ = [
    'DEBUG',
    'ERROR',
    'FATAL',
    'FileHandler',
    'Formatter',
    'getLogger',
    'getLogFileHandler',
    'getLogFilePath',
    'getOutputLoggers',
    'getScreenHandler',
    'Handler',
    'handlers',
    'INFO',
    'launchConfig',
    'Logger',
    'NOTSET',
    'reset',
    'root',
    'shutdown',
    'setLoggerClass',
    'StreamHandler',
    'WARN'
]


def with_per_logger_formatting(cls):
    """Add per logger formatting capabilities to the given logging.Handler."""
    class _trait(cls):
        """A logging.Handler subclass to enable per logger formatting."""

        def __init__(self, *args, **kwargs):
            super(_trait, self).__init__(*args, **kwargs)
            self._formatters = {}

        def setFormatterFor(self, logger, formatter):
            """Set formatter for a given logger instance or logger name."""
            logger_name = logger if isinstance(logger, str) else logger.name
            self._formatters[logger_name] = formatter

        def unsetFormatterFor(self, logger):
            """Unset formatter for a given logger instance or logger name, if any."""
            logger_name = logger if isinstance(logger, str) else logger.name
            if logger_name in self._formatters:
                del self._formatters[logger_name]

        def format(self, record):  # noqa
            if record.name in self._formatters:
                formatter = self._formatters[record.name]
                return formatter.format(record)
            return super(_trait, self).format(record)
    return _trait


def attributes(**attr):
    """Inject attributes into a function (a singleton by definition)."""
    def _decorator(f):
        for name, value in attr.items():
            setattr(f, name, value)
        return f
    return _decorator


@attributes(screen_handler=None, file_handlers={})
def launchConfig(
    *,
    level=None,
    log_dir=None,
    screen_format=None,
    screen_style=None,
    log_format=None,
    log_style=None
):
    """
    Set up launch logging.

    This function allows to:

    - Set the default verbosity level for all loggers.
    - Configure the location of log files on disk.
    - Configure screen and log file formats.

    Setup only has side effects for the arguments provided. The setup process is
    idempotent.

    :param: level to use as the default for all loggers.
    :param: log_dir to use as base path for all log file collections.
    :param: screen_format for logging to the screen, as expected by the
    `logging.Formatter` constructor. Alternatively, aliases for common
    formats are available, namely: 'default' to log verbosity level, logger
    name and logged message or 'default_with_timestamp' to add timestamps to
    the 'default' format.
    :param: screen_style the screen format style. No style can be provided
    if a format alias is given.
    :param: log_format for logging to the main launch log file, as expected
    by the `logging.Formatter` constructor. Alternatively, the 'default' alias
    can be given to log verbosity level, logger name and logged message.
    :param: log_style the log format style. No style can be provided if a format
    alias is given.
    """
    if level is not None:
        root.setLevel(level)
    if screen_format is not None:
        if screen_format == 'default':
            screen_format = '[{levelname}] [{name}]: {msg}'
            if screen_style is not None:
                raise ValueError('Cannot set a custom format style '
                                 'for the "default" screen format.')
        if screen_format == 'default_with_timestamp':
            screen_format = '{created:.7f} [{levelname}] [{name}]: {msg}'
            if screen_style is not None:
                raise ValueError('Cannot set a custom format style '
                                 'for the "default_with_timestamp" '
                                 'screen format.')
        if screen_style is None:
            screen_style = '{'
        launchConfig.screen_formatter = Formatter(
            screen_format, style=screen_style
        )
        if launchConfig.screen_handler is not None:
            launchConfig.screen_handler.setFormatter(launchConfig.screen_formatter)
    if log_format is not None:
        if log_format == 'default':
            log_format = '{created:.7f} [{levelname}] [{name}]: {msg}'
            if log_style is not None:
                raise ValueError('Cannot set a custom format style '
                                 'for the "default" log format.')
        if log_style is None:
            log_style = '{'
        launchConfig.file_formatter = Formatter(
            log_format, style=log_style
        )
        for handler in launchConfig.file_handlers.values():
            handler.setFormatter(launchConfig.file_formatter)
    if log_dir is not None:
        if any(launchConfig.file_handlers):
            import warnings
            warnings.warn(('Loggers have been already configured '
                           'to output to log files below {}. Proceed '
                           'at your own risk').format(launchConfig.log_dir))
        if not os.path.isdir(log_dir):
            raise ValueError('{} is not a directory'.format(log_dir))
        launchConfig.log_dir = log_dir


def getLogger(name=None):
    """Get named logger, configured to output to screen and launch main log file."""
    import logging
    logger = logging.getLogger(name)
    screen_handler = getScreenHandler()
    if screen_handler not in logger.handlers:
        logger.addHandler(screen_handler)
    launch_log_file_handler = getLogFileHandler()
    if launch_log_file_handler not in logger.handlers:
        logger.addHandler(launch_log_file_handler)
    return logger


def _normalize_output_configuration(config):
    """
    Normalize output configuration to a dict representation.

    See getOutputLoggers() documentation for further reference.
    """
    normalized_config = {
        'both': set(), 'stdout': set(), 'stderr': set()
    }
    if isinstance(config, str):
        if config == 'screen':
            normalized_config.update({
                'both': {'screen'}
            })
        elif config == 'log':
            normalized_config.update({
                'both': {'log'},
                'stderr': {'screen'}
            })
        elif config == 'both':
            normalized_config.update({
                'both': {'log', 'screen'},
            })
        elif config == 'own_log':
            normalized_config.update({
                'both': {'own_log'},
                'stdout': {'own_log'},
                'stderr': {'own_log'}
            })
        elif config == 'full':
            normalized_config.update({
                'both': {'screen', 'log', 'own_log'},
                'stdout': {'own_log'},
                'stderr': {'own_log'}
            })
        else:
            raise ValueError((
                '{} is not a valid standard output config '
                'i.e. "screen", "log" or "both"'
            ).format(config))
    elif isinstance(config, dict):
        for source, destinations in config.items():
            if source not in ('stdout', 'stderr', 'both'):
                raise ValueError((
                    '{} is not a valid output source '
                    'i.e. "stdout", "stderr" or "both"'
                ).format(source))
            if isinstance(destinations, str):
                destinations = {destinations}
            for destination in destinations:
                if destination not in ('screen', 'log', 'own_log'):
                    raise ValueError((
                        '{} is not a valid output destination '
                        'i.e. "screen", "log" or "own_log"'
                    ).format(destination))
            normalized_config[source] = set(destinations)
    else:
        raise ValueError(
            '{} is not a valid output configuration'.format(config)
        )
    return normalized_config


def getOutputLoggers(proc_name, output_config):
    """
    Get the stdout and stderr output loggers for the given process-like action.

    :param: proc_action whose outputs want to be logged.
    :param: output_config for the output loggers i.e. a dictionary with optional 'stdout',
    'stderr' and 'both' (stdout and stderr combined) process output sources to a set of one
    or more logging destinations, namely 'screen' to log it to the screen, 'log' to log it
    to launch main log file and 'own_log' to log it to a separate log file. Separate log file
    names follow the `<proc_name>-<source>.log` pattern, except when the source is 'both' in
    which case the log file name follows the `<proc_name>.log` pattern. Alternatively, aliases
    for common configurations are available, namely: 'screen' for both stdout and stderr to be
    logged to the screen, 'log' for both stdout and stderr to be logged to launch main log file
    and stderr to the screen, 'both' for both stdout and stderr to be logged to the screen and
    to launch main log file, 'own_log' for stdout, stderr and their combination to be logged to
    their own log files, and 'full' to have stdout and stderr sent to the screen, to the main
    launch log file, and their own separate and combined log files.
    :returns: a tuple with the stdout and stderr output loggers.
    """
    import logging
    output_config = _normalize_output_configuration(output_config)
    for source in ('stdout', 'stderr'):
        logger = logging.getLogger('{}-{}'.format(proc_name, source))
        # If a 'screen' output is configured for this source or for
        # 'both' sources, this logger should output to screen.
        if 'screen' in (output_config['both'] | output_config[source]):
            screen_handler = getScreenHandler()
            # Add screen handler if necessary.
            if screen_handler not in logger.handlers:
                screen_handler.setFormatterFor(
                    logger, Formatter('{msg}', style='{')
                )
                logger.addHandler(screen_handler)

        # If a 'log' output is configured for this source or for
        # 'both' sources, this logger should output to launch main log file.
        if 'log' in (output_config['both'] | output_config[source]):
            launch_log_file_handler = getLogFileHandler()
            # Add launch main log file handler if necessary.
            if launch_log_file_handler not in logger.handlers:
                launch_log_file_handler.setFormatterFor(
                    logger, Formatter('{created:.7f} {msg}', style='{')
                )
                logger.addHandler(launch_log_file_handler)

        # If an 'own_log' output is configured for this source, this logger
        # should output to its own log file.
        if 'own_log' in output_config[source]:
            own_log_file_handler = getLogFileHandler(
                '{}-{}.log'.format(proc_name, source)
            )
            own_log_file_handler.setFormatter(Formatter(fmt=None))
            # Add own log file handler if necessary.
            if own_log_file_handler not in logger.handlers:
                logger.addHandler(own_log_file_handler)
        # If an 'own_log' output is configured for 'both' sources,
        # this logger should output to a combined log file.
        if 'own_log' in output_config['both']:
            combined_log_file_handler = getLogFileHandler(proc_name + '.log')
            combined_log_file_handler.setFormatter(Formatter('{msg}', style='{'))
            # Add combined log file handler if necessary.
            if combined_log_file_handler not in logger.handlers:
                logger.addHandler(combined_log_file_handler)
    # Retrieve both loggers.
    return (logging.getLogger(proc_name + '-stdout'),
            logging.getLogger(proc_name + '-stderr'))


def getScreenHandler():
    """
    Get the one and only screen logging handler.

    See setup() documentation for screen logging configuration
    details.
    """
    if launchConfig.screen_handler is None:
        handler_cls = with_per_logger_formatting(StreamHandler)
        launchConfig.screen_handler = handler_cls(sys.stdout)
        launchConfig.screen_handler.setFormatter(launchConfig.screen_formatter)
    return launchConfig.screen_handler


def getLogFilePath(file_name='launch.log'):
    return os.path.join(launchConfig.log_dir, file_name)


def getLogFileHandler(file_name='launch.log'):
    """
    Get the logging handler to a log file.

    See setup() documentation for application wide log file logging
    configuration.

    :param: file_name of the log file whose handler is to be retrieved.
    :return: the logging handler associated to the file (always the same
    once constructed).
    """
    if file_name not in launchConfig.file_handlers:
        file_path = getLogFilePath(file_name)
        if os.name != 'nt':
            handler_cls = with_per_logger_formatting(
                handlers.WatchedFileHandler
            )
        else:
            handler_cls = with_per_logger_formatting(FileHandler)
        file_handler = handler_cls(file_path)
        file_handler.setFormatter(launchConfig.file_formatter)
        launchConfig.file_handlers[file_name] = file_handler
    return launchConfig.file_handlers[file_name]


def _make_unique_log_dir(*, base_path):
    """
    Make a unique directory for logging.

    :param: base_path for directory creation
    :return: the path to the created directory
    """
    while True:
        now = datetime.datetime.now()
        datetime_str = now.strftime('%Y-%m-%d-%H-%M-%S-%f')
        log_dirname = '{0}-{1}-{2}'.format(
            datetime_str, socket.gethostname(), os.getpid()
        )
        log_dir = os.path.join(base_path, log_dirname)
        # Check that filename does not exist
        # TODO(hidmic): fix (unlikely) TOCTTOU race
        if not os.path.isdir(log_dir):
            os.makedirs(log_dir, exist_ok=True)
            return log_dir


# Track all loggers to support module resets
class Logger(Logger):
    all_loggers = []

    def __new__(cls, *args, **kwargs):
        instance = super(Logger, cls).__new__(cls)
        Logger.all_loggers.append(instance)
        return instance


default_log_dir = _make_unique_log_dir(
    base_path=os.path.join(os.path.expanduser('~'), '.ros/log')
)


def reset():
    """Reset logging."""
    # Reset existing logging infrastructure
    for logger in Logger.all_loggers:
        logger.setLevel(NOTSET)
        del logger.handlers[:]
    # Back to default logging setup
    launchConfig.log_dir = None
    launchConfig.file_handlers = {}
    launchConfig.screen_handler = None
    launchConfig(
        level=INFO, log_dir=default_log_dir,
        log_format='default', screen_format='default'
    )
    setLoggerClass(Logger)


# Initial module reset
reset()
