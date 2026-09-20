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


def ready_to_test_action_timeout(timeout):
    """
    Decorate a test launch description in a way that it adds ReadyToTest action timeout.

    attribute to the function being decorated.

    :param: timeout Duration for which the ReadyToTest action waits for processes to start up

    """

    def _decorator(func):
        func.__ready_to_test_action_timeout__ = timeout
        return func

    return _decorator
