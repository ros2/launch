# Copyright 2022 Open Source Robotics Foundation, Inc.
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

import functools
import os


class TemporaryEnvironment:
    """Allow temporary changes to environment variables."""

    def __init__(self):
        self.__old_env = None

    def __enter__(self):
        self.__old_env = os.environ.copy()
        return self

    def __exit__(self, t, v, tb):
        # Update keys manually to make sure putenv gets called
        for key, value in self.__old_env.items():
            os.environ[key] = value
        # Remove added keys
        env_keys = tuple(os.environ)
        for key in env_keys:
            if key not in self.__old_env:
                del os.environ[key]
        self.__old_env = None


def sandbox_environment_variables(func):
    """Decorate a function to give it a temporary environment."""
    @functools.wraps(func)
    def wrapper_func(*args, **kwargs):
        with TemporaryEnvironment():
            func(*args, **kwargs)

    return wrapper_func
