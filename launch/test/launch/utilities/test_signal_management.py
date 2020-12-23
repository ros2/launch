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
import signal

from launch.utilities import AsyncSafeSignalManager

import osrf_pycommon.process_utils


def test_async_safe_signal_manager():
    """Test AsyncSafeSignalManager class."""
    loop = osrf_pycommon.process_utils.get_loop()

    prev_signal_handler = signal.signal(
        signal.SIGUSR1, lambda signum: None
    )
    try:
        got_signal = asyncio.Future(loop=loop)
        with AsyncSafeSignalManager(loop) as manager:
            manager.handle(
                signal.SIGUSR1,
                lambda signum: got_signal.set_result(signum)
            )
            loop.call_soon(signal.raise_signal, signal.SIGUSR1)
            loop.run_until_complete(
                asyncio.wait([got_signal], timeout=5.0)
            )
            assert got_signal.done()
            assert got_signal.result() == signal.SIGUSR1
    finally:
        signal.signal(signal.SIGUSR1, prev_signal_handler)
