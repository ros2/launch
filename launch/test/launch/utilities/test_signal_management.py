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

from launch.utilities import install_signal_handlers, on_sigint, on_sigquit, on_sigterm

import pytest


def test_install_signal_handlers():
    """Test the install_signal_handlers() function."""
    install_signal_handlers()
    install_signal_handlers()
    install_signal_handlers()


def test_on_sigint():
    """Test the on_sigint() function."""
    # None is acceptable
    on_sigint(None)

    def mock_sigint_handler():
        pass

    on_sigint(mock_sigint_handler)

    # Non-callable is not
    with pytest.raises(ValueError):
        on_sigint('non-callable')

    # TODO(jacobperron): implement a functional test by using subprocess.Popen


def test_on_sigquit():
    """Test the on_sigquit() function."""
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
    # None is acceptable
    on_sigterm(None)

    def mock_sigterm_handler():
        pass

    on_sigterm(mock_sigterm_handler)

    # Non-callable is not
    with pytest.raises(ValueError):
        on_sigterm('non-callable')

    # TODO(jacobperron): implement a functional test by using subprocess.Popen
