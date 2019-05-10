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

"""Tests for the launch.logging module."""

import logging
import os
import re

import launch.logging

import pytest


@pytest.fixture
def log_dir(tmpdir_factory):
    """Test fixture that generates a temporary directory for log files."""
    return str(tmpdir_factory.mktemp('logs'))


def test_bad_logging_launch_config():
    """Tests that setup throws at bad configuration."""
    launch.logging.reset()

    with pytest.raises(ValueError):
        launch.logging.launch_config(log_dir='not/a/real/dir')

    with pytest.raises(ValueError):
        launch.logging.launch_config(screen_format='default', screen_style='%')

    with pytest.raises(ValueError):
        launch.logging.launch_config(log_format='default', log_style='%')


def test_output_loggers_bad_configuration(log_dir):
    """Tests that output loggers setup throws at bad configuration."""
    launch.logging.reset()

    launch.logging.launch_config(log_dir=log_dir)

    with pytest.raises(ValueError):
        launch.logging.get_output_loggers('some-proc', 'not_an_alias')

    with pytest.raises(ValueError):
        launch.logging.get_output_loggers('some-proc', {'garbage': {'log'}})

    with pytest.raises(ValueError):
        launch.logging.get_output_loggers('some-proc', {'stdout': {'garbage'}})


@pytest.mark.parametrize('config,checks', [
    ('screen', {'stdout': {'screen'}, 'stderr': {'screen'}}),
    ('log', {'stdout': {'log'}, 'stderr': {'log', 'screen'}}),
    ('both', {'both': {'log', 'screen'}}),
    ('own_log', {
        'stdout': {'own_log'},
        'stderr': {'own_log'},
        'both': {'own_log'},
    }),
    ('full', {
        'stdout': {'log', 'own_log', 'screen'},
        'stderr': {'log', 'own_log', 'screen'},
        'both': {'own_log'},
    }),
    (
        {'stdout': {'screen', 'log'}, 'stderr': {'own_log'}},
        {
            'stdout': {'screen', 'log'},
            'stderr': {'own_log'}
        },
    )
])
def test_output_loggers_configuration(capsys, log_dir, config, checks):
    checks = {'stdout': set(), 'stderr': set(), 'both': set(), **checks}
    launch.logging.reset()
    launch.logging.launch_config(
        level=logging.INFO,
        log_dir=log_dir,
        screen_format='default',
        log_format='default'
    )
    logger = launch.logging.get_logger('some-proc')
    logger.addHandler(launch.logging.get_screen_handler())
    logger.addHandler(launch.logging.get_log_file_handler())
    logger.setLevel(logging.ERROR)
    stdout_logger, stderr_logger = launch.logging.get_output_loggers('some-proc', config)

    logger.debug('oops')
    logger.error('baz')
    stdout_logger.info('foo')
    stderr_logger.info('bar')

    capture = capsys.readouterr()
    lines = list(reversed(capture.out.splitlines()))
    assert '[ERROR] [some-proc]: baz' == lines.pop()
    if 'screen' in (checks['stdout'] | checks['both']):
        assert 'foo' == lines.pop()
    if 'screen' in (checks['stderr'] | checks['both']):
        assert 'bar' == lines.pop()
    assert 0 == len(lines)
    assert 0 == len(capture.err)

    launch.logging.get_log_file_handler().flush()
    main_log_path = launch.logging.get_log_file_path()
    assert os.path.exists(main_log_path)
    assert 0 != os.stat(main_log_path).st_size
    with open(main_log_path, 'r') as f:
        lines = list(reversed(f.readlines()))
        assert re.match(r'[0-9]+\.[0-9]+ \[ERROR\] \[some-proc\]: baz', lines.pop()) is not None
        if 'log' in (checks['stdout'] | checks['both']):
            assert re.match(r'[0-9]+\.[0-9]+ foo', lines.pop()) is not None
        if 'log' in (checks['stderr'] | checks['both']):
            assert re.match(r'[0-9]+\.[0-9]+ bar', lines.pop()) is not None
        assert 0 == len(lines)

    if 'own_log' in (checks['stdout'] | checks['both']):
        launch.logging.get_log_file_handler('some-proc-stdout.log').flush()
        own_log_path = launch.logging.get_log_file_path('some-proc-stdout.log')
        assert os.path.exists(own_log_path)
        assert 0 != os.stat(own_log_path).st_size
        with open(own_log_path, 'r') as f:
            lines = f.read().splitlines()
            assert 1 == len(lines)
            assert 'foo' == lines[0]
    else:
        own_log_path = launch.logging.get_log_file_path('some-proc-stdout.log')
        assert (not os.path.exists(own_log_path) or 0 == os.stat(own_log_path).st_size)

    if 'own_log' in (checks['stderr'] | checks['both']):
        launch.logging.get_log_file_handler('some-proc-stderr.log').flush()
        own_log_path = launch.logging.get_log_file_path('some-proc-stderr.log')
        assert os.path.exists(own_log_path)
        assert 0 != os.stat(own_log_path).st_size
        with open(own_log_path, 'r') as f:
            lines = f.read().splitlines()
            assert 1 == len(lines)
            assert 'bar' == lines[0]
    else:
        own_log_path = launch.logging.get_log_file_path('some-proc-stderr.log')
        assert (not os.path.exists(own_log_path) or 0 == os.stat(own_log_path).st_size)

    if 'own_log' in checks['both']:
        launch.logging.get_log_file_handler('some-proc.log').flush()
        own_log_path = launch.logging.get_log_file_path('some-proc.log')
        assert os.path.exists(own_log_path)
        assert 0 != os.stat(own_log_path).st_size
        with open(own_log_path, 'r') as f:
            lines = f.read().splitlines()
            assert 2 == len(lines)
            assert 'foo' == lines[0]
            assert 'bar' == lines[1]
    else:
        own_log_path = launch.logging.get_log_file_path('some-proc.log')
        assert (not os.path.exists(own_log_path) or 0 == os.stat(own_log_path).st_size)


def test_screen_default_format_with_timestamps(capsys, log_dir):
    """Test screen logging when using the default logs format with timestamps."""
    launch.logging.reset()
    launch.logging.launch_config(
        level=logging.DEBUG,
        log_dir=log_dir,
        screen_format='default_with_timestamp',
    )
    logger = launch.logging.get_logger('some-proc')
    logger.addHandler(launch.logging.get_screen_handler())
    assert logger.getEffectiveLevel() == logging.DEBUG

    logger.debug('foo')

    capture = capsys.readouterr()
    lines = capture.out.splitlines()
    assert 1 == len(lines)
    assert re.match(r'[0-9]+\.[0-9]+ \[DEBUG\] \[some-proc\]: foo', lines[0]) is not None
    assert 0 == len(capture.err)


def test_screen_default_format(capsys):
    """Test screen logging when using the default logs format."""
    launch.logging.reset()
    launch.logging.launch_config(
        level=logging.INFO,
        screen_format='default'
    )

    logger = launch.logging.get_logger('some-proc')
    logger.addHandler(launch.logging.get_screen_handler())
    assert logger.getEffectiveLevel() == logging.INFO

    logger.info('bar')
    capture = capsys.readouterr()
    lines = capture.out.splitlines()
    assert 1 == len(lines)
    assert '[INFO] [some-proc]: bar' == lines[0]
    assert 0 == len(capture.err)


def test_log_default_format(log_dir):
    """Test logging to the main log file when using the default logs format."""
    launch.logging.reset()
    launch.logging.launch_config(
        level=logging.WARN,
        log_dir=log_dir,
        log_format='default'
    )
    logger = launch.logging.get_logger('some-proc')
    logger.addHandler(launch.logging.get_log_file_handler())
    assert logger.getEffectiveLevel() == logging.WARN

    logger.error('baz')

    launch.logging.get_log_file_handler().flush()
    assert os.path.exists(launch.logging.get_log_file_path())
    assert 0 != os.stat(launch.logging.get_log_file_path()).st_size

    with open(launch.logging.get_log_file_path(), 'r') as f:
        lines = f.readlines()
        assert 1 == len(lines)
        assert re.match(r'[0-9]+\.[0-9]+ \[ERROR\] \[some-proc\]: baz', lines[0]) is not None


def test_log_handler_factory(log_dir):
    class TestStreamHandler(launch.logging.handlers.Handler):

        def __init__(self, output):
            super().__init__()
            self._output = output

        def emit(self, record):
            self._output.append(self.format(record))

    import collections
    outputs = collections.defaultdict(list)

    launch.logging.reset()
    launch.logging.launch_config(
        level=logging.WARN,
        log_dir=log_dir,
        log_format='default',
        log_handler_factory=(
            lambda path: TestStreamHandler(
                output=outputs[path]
            )
        )
    )

    logger = launch.logging.get_logger('some-proc')
    logger.addHandler(launch.logging.get_log_file_handler())

    logger.debug('foo')
    logger.error('baz')

    path = launch.logging.get_log_file_path()
    assert path in outputs
    assert len(outputs[path]) == 1
    assert outputs[path][0].endswith('baz')
