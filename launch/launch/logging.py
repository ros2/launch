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
import logging
import logging.handlers

import os
import socket
import sys

from typing import List


__all__ = [
    'get_logger',
    'get_log_file_handler',
    'get_log_file_path',
    'get_output_loggers',
    'get_screen_handler',
    'launch_config',
    'reset',
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
def launch_config(
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

    This function allows you to:

      - Set the default verbosity level for all loggers.
      - Configure the location of log files on disk.
      - Configure screen and log file formats.

    Setup only has side effects for the arguments provided.
    The setup process is idempotent.

    For the ``screen_format`` argument there are a few aliases:

      - 'default' to log verbosity level, logger name and logged message
      - 'default_with_timestamp' to add timestamps to the 'default' format

    :param level: the default log level used for all loggers.
    :param log_dir: used as base path for all log file collections.
    :param screen_format: format specification used when logging to the screen,
        as expected by the `logging.Formatter` constructor.
        Alternatively, aliases for common formats are available, see above.
    :param screen_style: the screen style used if no alias is used for
        screen_format.
        No style can be provided if a format alias is given.
    :param log_format: the format used when logging to the main launch log file,
        as expected by the `logging.Formatter` constructor.
        Alternatively, the 'default' alias can be given to log verbosity level,
        logger name and logged message.
    :param log_style: the log style used if no alias is given for log_format.
        No style can be provided if a format alias is given.
    """
    if level is not None:
        logging.root.setLevel(level)
    if screen_format is not None:
        if screen_format == 'default':
            screen_format = '[{levelname}] [{name}]: {msg}'
            if screen_style is not None:
                raise ValueError(
                    'Cannot set a custom format style for the "default" screen format.'
                )
        if screen_format == 'default_with_timestamp':
            screen_format = '{created:.7f} [{levelname}] [{name}]: {msg}'
            if screen_style is not None:
                raise ValueError(
                    'Cannot set a custom format style for the '
                    '"default_with_timestamp" screen format.'
                )
        if screen_style is None:
            screen_style = '{'
        launch_config.screen_formatter = logging.Formatter(
            screen_format, style=screen_style
        )
        if launch_config.screen_handler is not None:
            launch_config.screen_handler.setFormatter(launch_config.screen_formatter)
    if log_format is not None:
        if log_format == 'default':
            log_format = '{created:.7f} [{levelname}] [{name}]: {msg}'
            if log_style is not None:
                raise ValueError(
                    'Cannot set a custom format style for the "default" log format.'
                )
        if log_style is None:
            log_style = '{'
        launch_config.file_formatter = logging.Formatter(
            log_format, style=log_style
        )
        for handler in launch_config.file_handlers.values():
            handler.setFormatter(launch_config.file_formatter)
    if log_dir is not None:
        if any(launch_config.file_handlers):
            import warnings
            warnings.warn((
                'Loggers have been already configured to output to log files below {}. '
                'Proceed at your own risk.'
            ).format(launch_config.log_dir))
        if not os.path.isdir(log_dir):
            raise ValueError('{} is not a directory'.format(log_dir))
        launch_config.log_dir = log_dir


def get_logger(name=None):
    """Get named logger, configured to output to screen and launch main log file."""
    logger = logging.getLogger(name)
    screen_handler = get_screen_handler()
    if screen_handler not in logger.handlers:
        logger.addHandler(screen_handler)
    launch_log_file_handler = get_log_file_handler()
    if launch_log_file_handler not in logger.handlers:
        logger.addHandler(launch_log_file_handler)
    return logger


def _normalize_output_configuration(config):
    """
    Normalize output configuration to a dict representation.

    See `get_output_loggers()` documentation for further reference.
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


def get_output_loggers(process_name, output_config):
    """
    Get the stdout and stderr output loggers for the given process name.

    The output_config may be a dictionary with one or more of the optional keys
    'stdout', 'stderr', or 'both' (stdout and stderr combined) which represent
    the various process output sources, and values for those keys to assign one
    or more logging destinations to the source.
    The logging destination values may be:

      - 'screen': log it to the screen,
      - 'log': log it to launch log file, or
      - 'own_log': log it to a separate log file.

    When logging the stdout and stderr separately, the log file names follow
    the ``<process_name>-<source>.log`` pattern where ``<source>`` is either
    'stdout' or 'stderr'
    When the 'both' logging destination is used the log file name follows the
    ``<process_name>.log`` pattern.

    The "launch log file" is a log file which is create for each run of
    the launch.LaunchService, and at least captures the log output from launch
    itself, but may also include output from subprocess's if configured so.

    Alternatively, the output_config parameter may be a string which represents
    one of a couple available aliases for common logging configurations.
    The available aliases are:

      - 'screen': stdout and stderr are logged to the screen,
      - 'log': stdout and stderr are logged to launch log file and stderr to
            the screen,
      - 'both': both stdout and stderr are logged to the screen and to launch
            main log file,
      - 'own_log' for stdout, stderr and their combination to be logged to
            their own log files, and
      - 'full' to have stdout and stderr sent to the screen, to the main launch
            log file, and their own separate and combined log files.

    :param process_name: the process-like action whose outputs want to be logged.
    :param output_config: configuration for the output loggers,
        see above for details.
    :returns: a tuple with the stdout and stderr output loggers.
    """
    output_config = _normalize_output_configuration(output_config)
    for source in ('stdout', 'stderr'):
        logger = logging.getLogger('{}-{}'.format(process_name, source))
        # If a 'screen' output is configured for this source or for
        # 'both' sources, this logger should output to screen.
        if 'screen' in (output_config['both'] | output_config[source]):
            screen_handler = get_screen_handler()
            # Add screen handler if necessary.
            if screen_handler not in logger.handlers:
                screen_handler.setFormatterFor(
                    logger, logging.Formatter('{msg}', style='{')
                )
                logger.addHandler(screen_handler)

        # If a 'log' output is configured for this source or for
        # 'both' sources, this logger should output to launch main log file.
        if 'log' in (output_config['both'] | output_config[source]):
            launch_log_file_handler = get_log_file_handler()
            # Add launch main log file handler if necessary.
            if launch_log_file_handler not in logger.handlers:
                launch_log_file_handler.setFormatterFor(
                    logger, logging.Formatter('{created:.7f} {msg}', style='{')
                )
                logger.addHandler(launch_log_file_handler)

        # If an 'own_log' output is configured for this source, this logger
        # should output to its own log file.
        if 'own_log' in output_config[source]:
            own_log_file_handler = get_log_file_handler(
                '{}-{}.log'.format(process_name, source)
            )
            own_log_file_handler.setFormatter(logging.Formatter(fmt=None))
            # Add own log file handler if necessary.
            if own_log_file_handler not in logger.handlers:
                logger.addHandler(own_log_file_handler)
        # If an 'own_log' output is configured for 'both' sources,
        # this logger should output to a combined log file.
        if 'own_log' in output_config['both']:
            combined_log_file_handler = get_log_file_handler(process_name + '.log')
            combined_log_file_handler.setFormatter(logging.Formatter('{msg}', style='{'))
            # Add combined log file handler if necessary.
            if combined_log_file_handler not in logger.handlers:
                logger.addHandler(combined_log_file_handler)
    # Retrieve both loggers.
    return (
        logging.getLogger(process_name + '-stdout'),
        logging.getLogger(process_name + '-stderr')
    )


def get_screen_handler():
    """
    Get the one and only screen logging handler.

    See launch_config() documentation for screen logging configuration.
    """
    if launch_config.screen_handler is None:
        handler_cls = with_per_logger_formatting(logging.StreamHandler)
        launch_config.screen_handler = handler_cls(sys.stdout)
        launch_config.screen_handler.setFormatter(launch_config.screen_formatter)
    return launch_config.screen_handler


def get_log_file_path(file_name='launch.log'):
    return os.path.join(launch_config.log_dir, file_name)


def get_log_file_handler(file_name='launch.log'):
    """
    Get the logging handler to a log file.

    See launch_config() documentation for application wide log file
    logging configuration.

    :param: file_name of the log file whose handler is to be retrieved.
    :return: the logging handler associated to the file (always the same
    once constructed).
    """
    if file_name not in launch_config.file_handlers:
        file_path = get_log_file_path(file_name)
        if os.name != 'nt':
            handler_cls = with_per_logger_formatting(
                logging.handlers.WatchedFileHandler
            )
        else:
            handler_cls = with_per_logger_formatting(logging.FileHandler)
        file_handler = handler_cls(file_path)
        file_handler.setFormatter(launch_config.file_formatter)
        launch_config.file_handlers[file_name] = file_handler
    return launch_config.file_handlers[file_name]


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
class LaunchLogger(logging.getLoggerClass()):
    all_loggers: List[logging.Logger] = []

    def __new__(cls, *args, **kwargs):
        instance = super(LaunchLogger, cls).__new__(cls)
        LaunchLogger.all_loggers.append(instance)
        return instance


default_log_dir = _make_unique_log_dir(
    base_path=os.path.join(os.path.expanduser('~'), '.ros/log')
)


def reset():
    """Reset logging."""
    # Reset existing logging infrastructure
    for logger in LaunchLogger.all_loggers:
        logger.setLevel(logging.NOTSET)
        del logger.handlers[:]
    # Back to default logging setup
    launch_config.log_dir = None
    launch_config.file_handlers = {}
    launch_config.screen_handler = None
    launch_config(
        level=logging.INFO, log_dir=default_log_dir,
        log_format='default', screen_format='default'
    )
    logging.setLoggerClass(LaunchLogger)


# Initial module reset
reset()
