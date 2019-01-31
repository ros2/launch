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

"""Module for the LaunchLogger class."""

import datetime
import logging
from logging.handlers import WatchedFileHandler
import os
import socket
import sys
from typing import Optional
from typing import Text


class LoggerLevel:
    """Verbosity levels for loggers."""

    NOTSET = logging.NOTSET
    DEBUG = logging.DEBUG
    INFO = logging.INFO
    WARNING = logging.WARNING
    ERROR = logging.ERROR


class LaunchLogger:
    """
    Logging interface for launch and processes started by launch.

    This class enables:

    - Configuring logging to screen, to a log file, or both, for each process.
    - Setting verbosity levels for logs sent to the screen and log file for each process.
    - Setting a default verbosity level for all loggers.
    - Configuring the location of the log file on disk.

    This class is a singleton. The default verbosity level for all launch processes can
    be configured with multiple calls to the constructor.
    The log directory can only be configured with the first call to the constructor.
    """

    class __LevelFilter:

        def __init__(self, *, level):
            self.level = level
            self.names = set()

        def filter(self, record):
            return (not any(record.name.startswith(name) for name in self.names)
                    or record.levelno >= self.level)

    class __LaunchLogger:

        def __init__(self, *, level, log_dir, screen_timestamps=False):
            # Set the default verbosity level.
            logging.root.setLevel(level)

            # Generate log filename
            while True:
                datetime_str = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S-%f')
                log_basename = '{0}-{1}-{2}.log'.format(
                    datetime_str,
                    socket.gethostname(),
                    os.getpid(),
                )
                self.__log_filename = os.path.join(log_dir, log_basename)
                # Check that filename does not exist
                if not os.path.isfile(self.__log_filename):
                    break

            # Make sure directory exists
            os.makedirs(os.path.dirname(self.__log_filename), exist_ok=True)

            # Create handlers for the log file and screen
            # Use a 'WatchedFileHandler' in case file is modified (e.g. moved or removed)
            # This is not supported, nor necessary, in Windows
            if os.name != 'nt':
                self.file_handler = WatchedFileHandler(self.__log_filename)
            else:
                self.file_handler = logging.FileHandler(self.__log_filename)
            self.stdout_handler = logging.StreamHandler(sys.stdout)

            self.__stdout_filter = LaunchLogger.__LevelFilter(level=LoggerLevel.ERROR)
            self.stdout_handler.addFilter(self.__stdout_filter)

            # Establish formats for log file and screen
            # TODO(jacobperron): Add fixed padding to 'created' time
            self.__log_formatter = logging.Formatter(
                '{created:.7f} [{levelname}] [{name}]: {msg}',
                style='{',
            )
            screen_timestamp_format = '{created:.7f}' if screen_timestamps else ''
            self.__screen_formatter = logging.Formatter(
                screen_timestamp_format + '[{levelname}] [{name}]: {msg}',
                style='{',
            )
            self.file_handler.setFormatter(self.__log_formatter)
            self.stdout_handler.setFormatter(self.__screen_formatter)
            # TODO(jacobperron): Add a filter to the file handler for stripping
            #                    color escape sequences

        def configure_logger(self, name: Text, output: Text = 'log', level: Optional[int] = None):
            """
            Configure the output and verbosity level for a process by name.

            :param: name of the logger.
            :param: output type of the logger. Can be one of 'screen', 'log', or 'both';
                'screen' directs all logger output to the screen; 'log' directs all logger
                output to a log file and error logs only to the screen; 'both' directs all
                logger output to both the screen and a log file. Defaults to 'log'.
            :param: level of verbosity for the logger.
            """
            allowed_output_options = ['screen', 'log', 'both']
            if output not in allowed_output_options:
                raise ValueError(
                    "output argument to ExecuteProcess is '{}', expected one of [{}]".format(
                        output,
                        allowed_output_options,
                    )
                )
            logger = logging.getLogger(name)

            if level is not None:
                logger.setLevel(level)

            if 'log' == output:
                self.__stdout_filter.names.add(name)
            elif name in self.__stdout_filter.names:
                self.__stdout_filter.names.remove(name)

            # Avoid adding handlers more than once
            if self.file_handler not in logger.handlers:
                logger.addHandler(self.file_handler)
            if self.stdout_handler not in logger.handlers:
                logger.addHandler(self.stdout_handler)

        def log(self, name: Text, level: int, message: Text) -> None:
            """
            Log a message with the given level for a process or module.

            :param: name of the process or module.
            :param: level of verbosity for the message.
            :param: message to log.
            """
            logging.getLogger(name).log(level, message)

        def debug(self, name: Text, message: Text) -> None:
            """
            Log a debug message for a process or module.

            :param: name of the process or module.
            :param: message to log.
            """
            logging.getLogger(name).debug(message)

        def info(self, name: Text, message: Text) -> None:
            """
            Log an info message for a process or module.

            :param: name of the process or module.
            :param: message to log.
            """
            logging.getLogger(name).info(message)

        def warning(self, name: Text, message: Text) -> None:
            """
            Log a warning message for a process or module.

            :param: name of the process or module.
            :param: message to log.
            """
            logging.getLogger(name).warning(message)

        def error(self, name: Text, message: Text) -> None:
            """
            Log an error message for a process or module.

            :param: name of the process or module.
            :param: message to log.
            """
            logging.getLogger(name).error(message)

        @property
        def log_filename(self) -> Text:
            """Get the log filename."""
            return self.__log_filename

        def shutdown(self) -> None:
            """Perform an orderly shutdown, flushing and closing all handlers."""
            logging.shutdown()

    # Singleton instance
    instance = None

    def __init__(
        self,
        *,
        level: int = LoggerLevel.INFO,
        log_dir: Text = os.path.join(os.path.expanduser('~'), '.ros/log'),
        screen_timestamps: bool = False
    ):
        """
        Constructor.

        :param: level is the default verbosity level all processes. The level can changed with
            multiple calls to this constructor.
        :param: log_dir is where the launch log file will be created. The log directory can only
            be set once with the first call to this constructor.
        :param: screen_timestamps is a flag that if set to True, will prepend timestamps to log
            messages sent to the screen.
        """
        if LaunchLogger.instance is None:
            LaunchLogger.instance = LaunchLogger.__LaunchLogger(
                level=level,
                log_dir=log_dir,
                screen_timestamps=screen_timestamps,
            )
        else:
            logging.root.setLevel(level)

    def __getattr__(self, name):
        return getattr(LaunchLogger.instance, name)
