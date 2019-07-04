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
from typing import Iterable
from typing import List
from typing import Optional
from typing import Text
from typing import Type
from typing import Union

__types_for_guess = (
    int, float, bool, List[int], List[float],
    List[bool], List[str], str
)


AllowedTypes = Type[Union[__types_for_guess]]
SomeAllowedTypes = Union[AllowedTypes, Iterable[AllowedTypes]]


def extract_type(data_type: AllowedTypes):
    """
    Extract type information from type object.

    :param data_type: Can be one of:
        - `str`
        - `int`
        - `float`
        - `bool`
        - `List[str]`
        - `List[int]`
        - `List[float]`
        - `List[bool]`

    :returns: a tuple (type_obj, is_list).
        is_list is `True` for the supported list types, if not is `False`.
        type_obj is the object representing that type in python. In the case of list
        is the type of the items.
        e.g.:
            `name = List[int]` -> `(int, True)`
            `name = bool` -> `(bool, False)`
    """
    is_list = False
    if data_type not in __types_for_guess:
        raise ValueError('Unrecognized data type: {}'.format(data_type))
    if issubclass(data_type, List):
        is_list = True
        data_type = data_type.__args__[0]
    return (data_type, is_list)


def check_type(value: Any, types: Optional[SomeAllowedTypes]) -> bool:
    """
    Check if `value` is one of the types in `types`.

    The allowed types are:
        - `str`
        - `int`
        - `float`
        - `bool`
        - `List[str]`
        - `List[int]`
        - `List[float]`
        - `List[bool]`

    `types = None` works in the same way as:
        `(int, float, bool, List[int], List[float], List[bool], List[str], str)`
    """
    if types is None:
        types = __types_for_guess
    elif not isinstance(types, Iterable):
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


def get_typed_value(
    value: Union[Text, List[Text]],
    types: Optional[SomeAllowedTypes]
) -> Any:
    """
    Try to convert `value` to one of the types specified in `types`.

    It returns the first successful conversion.
    If not raise `AttributeError`.

    The allowed types are:
        - `str`
        - `int`
        - `float`
        - `bool`
        - `List[str]`
        - `List[int]`
        - `List[float]`
        - `List[bool]`

    `types = None` works in the same way as:
        `(int, float, bool, List[int], List[float], List[bool], List[str], str)`
    """
    if types is None:
        types = __types_for_guess
    elif types == str:
        # Avoid evaluating as yaml if types was just `str`
        return value
    elif not isinstance(types, Iterable):
        types = [types]

    value_is_list = isinstance(value, list)

    for x in types:
        type_obj, type_is_list = extract_type(x)
        if type_obj is bool:
            def type_obj(x):
                """Convert string to bool value."""
                if x.lower() in ('true', 'yes', 'on', '1', 'false', 'no', 'off', '0'):
                    return x.lower() in ('true', 'yes', 'on', '1')
                raise ValueError()
        elif type_obj is str:
            def type_obj(x):
                """Strip outer quotes if we have them."""
                if x.startswith("'") and x.endswith('"'):
                    return x[1:-1]
                elif x.startswith('"') and x.endswith('"'):
                    return x[1:-1]
                else:
                    return x
        if type_is_list != value_is_list:
            continue
        if type_is_list:
            try:
                return [type_obj(x) for x in value]
            except ValueError:
                pass
            else:
                break
        else:
            if isinstance(value, list):
                continue
            try:
                return type_obj(value)
            except ValueError:
                pass
    raise ValueError(
        'Can not convert value {} to one of the types in {}'.format(
            value, types
        )
    )
