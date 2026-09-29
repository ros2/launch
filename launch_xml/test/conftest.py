# Copyright 2026 Open Source Robotics Foundation, Inc.
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

"""Fixtures shared by all tests in this package."""

import os

import pytest


@pytest.fixture(autouse=True)
def sandbox_environment_variables():
    """
    Restore ``os.environ`` after each test.

    ``LaunchContext.environment`` is ``os.environ`` itself, so actions like
    ``ReplaceEnvironmentVariables`` and ``ResetEnvironment`` mutate the real process
    environment when a test executes them.  Without this fixture those mutations leak
    into every test that happens to run afterwards, which makes the suite sensitive to
    collection order.
    """
    saved_environment = os.environ.copy()
    try:
        yield
    finally:
        # Assign through os.environ rather than replacing it so that putenv() is
        # called for each key, keeping the C-level environment in sync.
        for key, value in saved_environment.items():
            os.environ[key] = value
        for key in tuple(os.environ):
            if key not in saved_environment:
                del os.environ[key]
