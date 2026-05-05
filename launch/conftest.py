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

import pytest


@pytest.hookimpl(tryfirst=True)
def pytest_ignore_collect(collection_path, config, path=None):
    # Pytest 7.x signature: (path, config)
    # Pytest 8.x signature: (collection_path, path, config)
    # By using (collection_path, config, path=None), we handle both:
    # 7.x: collection_path=path, config=config, path=None
    # 8.x: collection_path=collection_path, config=path, path=config
    # In both cases, the first argument 'collection_path' contains the path we care about.
    p = collection_path
    if p is None:
        return False

    p_str = str(p)

    # Ignore .launch.py files to avoid collection failures for launch_pytest tests
    if p_str.endswith('.launch.py'):
        return True

    # Ignore launch.logging.handlers to avoid collision with standard library
    # The file path typically ends with launch/logging/handlers.py or just logging/handlers.py
    if 'logging/handlers.py' in p_str.replace('\\', '/'):
        return True

    return False
