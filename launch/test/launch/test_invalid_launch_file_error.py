# Copyright 2024 Open Source Robotics Foundation, Inc.
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

from launch.invalid_launch_file_error import InvalidLaunchFileError


def test_invalid_launch_file_error():
    try:
        exception = KeyError('Test')
        raise InvalidLaunchFileError(extension='.py', likely_errors=[exception])
    except InvalidLaunchFileError as ex:
        assert 'KeyError' in ex.__str__()


def test_invalid_launch_file_errors():
    try:
        exceptions = [ValueError('Test1'), AttributeError('Test2'), BufferError('Test3')]
        raise InvalidLaunchFileError(extension='.py', likely_errors=exceptions)
    except InvalidLaunchFileError as ex:
        assert 'ValueError' in ex.__str__()
        assert 'AttributeError' in ex.__str__()
        assert 'BufferError' in ex.__str__()
