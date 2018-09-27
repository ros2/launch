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

"""Tests for the Launchlogger class."""

import os
import re

from launch import LaunchLogger, LoggerLevel

import pytest


@pytest.fixture(scope='session')
def log_directory(tmpdir_factory):
    """Test fixture that generates a temporary directory for log files."""
    log_directory = str(tmpdir_factory.mktemp('logs'))
    LaunchLogger(log_dir=log_directory)  # Create the singleton
    return log_directory


@pytest.fixture
def log_filename(log_directory):
    """Provide the log filename for each test and clear it after each test."""
    filename = LaunchLogger().log_filename
    yield filename
    # Overwrite any existing logs
    with open(filename, 'w'):
        pass


def test_launch_logger_constructors(log_filename):
    """Test the constructors for the LaunchLogger class."""
    LaunchLogger(level=100, log_dir='not/a/real/dir')
    LaunchLogger(level=50)
    LaunchLogger()


@pytest.mark.skipif(pytest.config.getoption('-s') != 'no',
                    reason='seems like "-s" option is required to capture stdout from logger')
def test_launch_logger_configure_logger_output(capfd, log_filename):
    """Test the configure_logger() methods output option for the LaunchLogger class."""
    log_pattern = '[0-9]+\.[0-9]+ \[INFO\] \[{}\] {}(\n|\r\n)'
    logger = LaunchLogger(level=LoggerLevel.DEBUG)
    logger.configure_logger(name='test_screen', output='screen')
    logger.info('test_screen', 'foo')
    capture = capfd.readouterr()
    assert re.match(log_pattern.format('test_screen', 'foo'), capture.out) is not None
    assert 0 == len(capture.err)
    assert 0 == os.stat(log_filename).st_size

    logger.configure_logger(name='test_log', output='log', level=LoggerLevel.INFO)
    logger.info('test_log', 'bar')
    capture = capfd.readouterr()
    assert 0 == len(capture.out)
    assert 0 == len(capture.err)
    with open(log_filename) as f:
        lines = f.readlines()
        assert 1 == len(lines)
        assert re.match(log_pattern.format('test_log', 'bar'), lines[0]) is not None

    logger.configure_logger(name='test_both', output='both', level=LoggerLevel.INFO)
    logger.info('test_both', 'foobar')
    capture = capfd.readouterr()
    assert re.match(log_pattern.format('test_both', 'foobar'), capture.out) is not None
    assert 0 == len(capture.err)
    with open(log_filename) as f:
        lines = f.readlines()
        # Now there are two lines in the log file b/c of the previous 'info'
        assert 2 == len(lines)
        assert re.match(log_pattern.format('test_log', 'bar'), lines[0]) is not None
        assert re.match(log_pattern.format('test_both', 'foobar'), lines[1]) is not None


@pytest.mark.skipif(pytest.config.getoption('-s') != 'no',
                    reason='seems like "-s" option is required to capture stdout from logger')
def test_launch_logger_configure_logger_level(capfd, log_filename):
    """Test the configure_logger() methods level option for the LaunchLogger class."""
    log_pattern = '[0-9]+\.[0-9]+ \[{}\] \[test_logger\] {}(\n|\r\n)'
    logger = LaunchLogger(level=LoggerLevel.ERROR)
    logger.configure_logger(name='test_logger', output='screen', level=LoggerLevel.DEBUG)
    logger.debug('test_logger', 'de bugs')
    capture = capfd.readouterr()
    assert re.match(log_pattern.format('DEBUG', 'de bugs'), capture.out) is not None
    logger.info('test_logger', 'info is okay')
    capture = capfd.readouterr()
    assert re.match(log_pattern.format('INFO', 'info is okay'), capture.out) is not None
    logger.warning('test_logger', 'warning too')
    capture = capfd.readouterr()
    assert re.match(log_pattern.format('WARNING', 'warning too'), capture.out) is not None
    logger.error('test_logger', 'danger will robinson')
    capture = capfd.readouterr()
    assert re.match(log_pattern.format('ERROR', 'danger will robinson'), capture.out) is not None

    logger.configure_logger(name='test_logger', level=LoggerLevel.WARNING)
    logger.debug('test_logger', 'no bugs allowed')
    capture = capfd.readouterr()
    assert 0 == len(capture.out)
    logger.info('test_logger', 'we dont want no information')
    capture = capfd.readouterr()
    assert 0 == len(capture.out)
    logger.warning('test_logger', 'your first warning')
    capture = capfd.readouterr()
    assert re.match(log_pattern.format('WARNING', 'your first warning'), capture.out) is not None
    logger.error('test_logger', 'errors are loud')
    capture = capfd.readouterr()
    assert re.match(log_pattern.format('ERROR', 'errors are loud'), capture.out) is not None


@pytest.mark.skipif(pytest.config.getoption('-s') != 'no',
                    reason='seems like "-s" option is required to capture stdout from logger')
def test_launch_logger_default_level(capfd, log_filename):
    """Test the default level of the LaunchLogger class is set and used correctly."""
    log_pattern = '[0-9]+\.[0-9]+ \[{}\] \[test_default\] {}(\n|\r\n)'
    # Set minimum level to INFO
    logger = LaunchLogger(level=LoggerLevel.INFO)
    logger.configure_logger(name='test_default', output='screen')
    logger.debug('test_default', 'default level is higher than this log')
    capture = capfd.readouterr()
    assert 0 == len(capture.out)
    logger.info('test_default', 'this is the minimum')
    capture = capfd.readouterr()
    assert re.match(log_pattern.format('INFO', 'this is the minimum'), capture.out) is not None

    # Reconfigure the minimum level
    logger = LaunchLogger(level=LoggerLevel.DEBUG)
    logger.debug('test_default', 'now debug gets through')
    capture = capfd.readouterr()
    assert re.match(log_pattern.format('DEBUG', 'now debug gets through'), capture.out) is not None

    # Set logger level higher than default
    logger.configure_logger(name='test_default', output='screen', level=LoggerLevel.WARNING)
    logger.warning('test_default', 'override')
    capture = capfd.readouterr()
    assert re.match(log_pattern.format('WARNING', 'override'), capture.out) is not None


def test_launch_logger_shutdown():
    """Test the shutdown() method for the LaunchLogger class."""
    logger = LaunchLogger()
    logger.shutdown()
