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

"""Module that provides methods for converting string to other data types."""

from typing import Iterable
from typing import Text
from typing import Union


def str_to_bool(string: Text):
    """Convert text to python bool."""
    if string.lower() in ('0', 'false'):
        return False
    if string.lower() in ('1', 'true'):
        return True
    raise RuntimeError('Expected "true" or "false", got {}'.format(string))


def guess_type_from_string(value: Union[Text, Iterable[Text]]):
    """Guess the desired type of the parameter based on the string value."""
    if not isinstance(value, Text):
        return [__guess_type_from_string(item) for item in value]
    return __guess_type_from_string(value)


def __guess_type_from_string(string_value: Text):
    if __is_bool(string_value):
        return string_value.lower() == 'true'
    if __is_integer(string_value):
        return int(string_value)
    if __is_float(string_value):
        return float(string_value)
    else:
        return string_value


def __is_bool(string_value: Text):
    if string_value.lower() in ('false', 'true'):
        return True
    return False


def __is_integer(string_value: Text):
    try:
        int(string_value)
    except ValueError:
        return False
    return True


def __is_float(string_value: Text):
    try:
        float(string_value)
    except ValueError:
        return False
    return True
