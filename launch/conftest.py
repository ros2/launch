# Copyright 2021 Open Source Robotics Foundation, Inc.
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

import pathlib

import pytest


def _pytest_version_ge(major, minor=0, patch=0):
    """Return True if pytest version is >= the given version."""
    pytest_version = tuple(int(v) for v in pytest.__version__.split('.'))
    return pytest_version >= (major, minor, patch)


def _should_ignore(p):
    if p is None:
        return False
    path = pathlib.Path(p)
    # Ignore .launch.py files — not valid Python module names.
    if path.name.endswith('.launch.py'):
        return True
    # Ignore launch.logging.handlers — collides with stdlib logging.handlers.
    if path.name == 'handlers.py' and path.parent.name == 'logging':
        return True
    return False


if _pytest_version_ge(8):
    def pytest_ignore_collect(collection_path, config):
        return _should_ignore(collection_path)
else:
    def pytest_ignore_collect(path, config):
        return _should_ignore(path)
