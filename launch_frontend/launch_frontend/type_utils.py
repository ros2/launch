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

"""Module which implements get_typed_value function."""

from typing import Any
from typing import Text
from typing import Tuple
from typing import Union


types_guess__ = (
    'int', 'float', 'bool', 'list[int]', 'list[float]',
    'list[bool]', 'list[str]', 'str'
)


def extract_type(name: Text):
    """
    Extract type information from string.

    `name` can be one of:
        - 'str'
        - 'int'
        - 'float'
        - 'bool'
        - 'list[str]'
        - 'list[int]'
        - 'list[float]'
        - 'list[bool]'

    Returns a tuple (type_obj, is_list).
    is_list is `True` for the supported list types, if not is `False`.
    type_obj is the object representing that type in python. In the case of list
    is the type of the items.
    e.g.:
        name = 'list[int]' -> (int, True)
        name = 'bool' -> (bool, False)
    """
    error = ValueError('Unrecognized type name: {}'.format(name))
    is_list = False
    type_name = name
    if 'list[' in name:
        is_list = True
        type_name = name[5:-1]
        if name[-1] != ']':
            raise error
    if type_name not in ('int', 'float', 'str', 'bool'):
        raise error
    return (eval(type_name), is_list)


def check_type(value: Any, types: Union[Text, Tuple[Text]]) -> bool:
    """
    Check if `value` is one of the types in `types`.

    The allowed types are:
        - 'str'
        - 'int'
        - 'float'
        - 'bool'
        - 'list[str]'
        - 'list[int]'
        - 'list[float]'
        - 'list[bool]'

    types = 'guess' works in the same way as:
        ('int', 'float', 'bool', 'list[int]', 'list[float]', 'list[bool]', 'list[str]', 'str')
    """
    if types == 'guess':
        types = types_guess__
    if isinstance(types, Text):
        types = [types]
    for x in types:
        type_obj, is_list = extract_type(x)
        if is_list:
            if not isinstance(value, list) or not value:
                continue
            if isinstance(value[0], type_obj):
                return True
        else:
            if isinstance(value, type_obj):
                return True
    return False


def get_typed_value(value: Text, types: Union[Text, Tuple[Text]]) -> Any:
    """
    Try to convert `value` to one of the types specified in `types`.

    It returns the first successful conversion.
    If not raise `AttributeError`.

    The allowed types are:
        - 'str'
        - 'int'
        - 'float'
        - 'bool'
        - 'list[str]'
        - 'list[int]'
        - 'list[float]'
        - 'list[bool]'

    types = 'guess' works in the same way as:
        ('int', 'float', 'bool', 'list[int]', 'list[float]', 'list[bool]', 'list[str]', 'str')
    """
    if types == 'guess':
        types = types_guess__
    elif isinstance(types, Text):
        types = [types]

    typed_value = None
    for x in types:
        type_obj, is_list = extract_type(x)
        if type_obj is bool:
            def type_obj(x):
                if x.lower() in ('true', 'false'):
                    return x.lower() == 'true'
                raise ValueError()
        if is_list:
            if not isinstance(value, list):
                continue
            try:
                typed_value = [type_obj(x) for x in value]
            except ValueError:
                pass
            else:
                break
        else:
            if isinstance(value, list):
                continue
            try:
                typed_value = type_obj(value)
            except ValueError:
                pass
            else:
                break
    if typed_value is None:
        raise ValueError(
            'Can not convert value {} to one of the types in {}'.format(
                value, types
            )
        )
    return typed_value
