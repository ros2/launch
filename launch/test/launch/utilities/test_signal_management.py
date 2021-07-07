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

"""Tests for the signal_management module."""

import asyncio
import functools
import platform
import signal

from launch.utilities import AsyncSafeSignalManager
from launch.utilities import install_signal_handlers
from launch.utilities import on_sigint
from launch.utilities import on_sigquit
from launch.utilities import on_sigterm

import osrf_pycommon.process_utils

import pytest


def cap_signals(*signals):
    def _noop(*args):
        pass

    def _decorator(func):
        @functools.wraps(func)
        def _wrapper(*args, **kwargs):
            handlers = {}
            try:
                for s in signals:
                    handlers[s] = signal.signal(s, _noop)
                return func(*args, **kwargs)
            finally:
                assert all(signal.signal(s, h) is _noop for s, h in handlers.items())
        return _wrapper

    return _decorator


if platform.system() == 'Windows':
    # NOTE(hidmic): this is risky, but we have few options.
    SIGNAL = signal.SIGINT
    ANOTHER_SIGNAL = signal.SIGBREAK
else:
    SIGNAL = signal.SIGUSR1
    ANOTHER_SIGNAL = signal.SIGUSR2

if not hasattr(signal, 'raise_signal'):
    # Only available for Python 3.8+
    def raise_signal(signum):
        import os
        os.kill(os.getpid(), signum)
else:
    raise_signal = signal.raise_signal


@cap_signals(SIGNAL, ANOTHER_SIGNAL)
def test_async_safe_signal_manager():
    """Test AsyncSafeSignalManager class."""
    loop = osrf_pycommon.process_utils.get_loop()

    manager = AsyncSafeSignalManager(loop)

    # Register signal handler outside context
    got_signal = asyncio.Future(loop=loop)
    manager.handle(SIGNAL, got_signal.set_result)

    # Signal handling is active within context
    with manager:
        # Register signal handler within context
        got_another_signal = asyncio.Future(loop=loop)
        manager.handle(ANOTHER_SIGNAL, got_another_signal.set_result)

        # Verify signal handling is working
        loop.call_soon(raise_signal, SIGNAL)
        loop.run_until_complete(asyncio.wait(
            [got_signal, got_another_signal],
            return_when=asyncio.FIRST_COMPLETED,
            timeout=1.0
        ))
        assert got_signal.done()
        assert got_signal.result() == SIGNAL
        assert not got_another_signal.done()

        # Unregister signal handler within context
        manager.handle(SIGNAL, None)

        # Verify signal handler is no longer there
        loop.call_soon(raise_signal, SIGNAL)
        loop.run_until_complete(asyncio.wait(
            [got_another_signal], timeout=1.0
        ))
        assert not got_another_signal.done()

    # Signal handling is (now) inactive outside context
    loop.call_soon(raise_signal, ANOTHER_SIGNAL)
    loop.run_until_complete(asyncio.wait(
        [got_another_signal], timeout=1.0
    ))
    assert not got_another_signal.done()

    # Managers' context may be re-entered
    with manager:
        loop.call_soon(raise_signal, ANOTHER_SIGNAL)
        loop.run_until_complete(asyncio.wait(
            [got_another_signal], timeout=1.0
        ))
        assert got_another_signal.done()
        assert got_another_signal.result() == ANOTHER_SIGNAL


def test_install_signal_handlers():
    """Test the install_signal_handlers() function."""
    with pytest.deprecated_call():
        install_signal_handlers()
        install_signal_handlers()
        install_signal_handlers()


def test_on_sigint():
    """Test the on_sigint() function."""
    with pytest.deprecated_call():
        # None is acceptable
        on_sigint(None)

        def mock_sigint_handler():
            pass

        on_sigint(mock_sigint_handler)

        # Non-callable is not
        with pytest.raises(ValueError):
            on_sigint('non-callable')

        # TODO(jacobperron): implement a functional test by using subprocess.Popen


@pytest.mark.skipif(platform.system() == 'Windows',
                    reason='No SIGQUIT on Windows')
def test_on_sigquit():
    """Test the on_sigquit() function."""
    with pytest.deprecated_call():
        # None is acceptable
        on_sigquit(None)

        def mock_sigquit_handler():
            pass

        on_sigquit(mock_sigquit_handler)

        # Non-callable is not
        with pytest.raises(ValueError):
            on_sigquit('non-callable')

        # TODO(jacobperron): implement a functional test by using subprocess.Popen


def test_on_sigterm():
    """Test the on_sigterm() function."""
    with pytest.deprecated_call():
        # None is acceptable
        on_sigterm(None)

        def mock_sigterm_handler():
            pass

        on_sigterm(mock_sigterm_handler)

        # Non-callable is not
        with pytest.raises(ValueError):
            on_sigterm('non-callable')

        # TODO(jacobperron): implement a functional test by using subprocess.Popen
