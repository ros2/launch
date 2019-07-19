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

import functools
import inspect


def keep_alive(test_description):
    """Mark a test launch description to be kept alive after fixture processes' termination."""
    if not hasattr(test_description, '__markers__'):
        test_description.__markers__ = {}
    test_description.__markers__['keep_alive'] = True
    return test_description


def retry_on_failure(*, times):
    """Mark a test case to be retried up to `times` on AssertionError."""
    assert times > 0

    def _decorator(func):
        @functools.wraps(func)
        def _wrapper(*args, **kwargs):
            n = times
            while n > 1:
                try:
                    return func(*args, **kwargs)
                except AssertionError:
                    n -= 1
            return func(*args, **kwargs)
        # Keep signature for argument binding
        _wrapper.__signature__ = inspect.signature(func)
        return _wrapper
    return _decorator
