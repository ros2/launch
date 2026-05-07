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


@pytest.hookimpl(tryfirst=True)
def pytest_ignore_collect(collection_path=None, path=None, config=None):
    # Pytest 7.x signature: (path, config)
    # Pytest 8.x signature: (collection_path, path, config)
    p = collection_path or path
    if p is None:
        return False

    path_obj = pathlib.Path(p)

    # Ignore .launch.py files to avoid collection failures for launch_pytest tests
    if path_obj.name.endswith('.launch.py'):
        return True

    # Ignore launch.logging.handlers to avoid collision with standard library
    # The file path typically ends with launch/logging/handlers.py or just logging/handlers.py
    if path_obj.name == 'handlers.py' and path_obj.parent.name == 'logging':
        return True

    return False
