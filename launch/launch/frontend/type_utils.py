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

__supported_types = (
    int, float, bool, str, List[str], List[int], List[float], List[bool], list, List
)

__scalar_types = (
    int, float, bool, str
)

__types_for_guess = (
    int, float, bool, list, str
)


AllowedTypes = Type[Union[__supported_types]]
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
        - `list` or `List`

    :returns: a tuple (type_obj, is_list).
        is_list is `True` for the supported list types, if not is `False`.
        type_obj is the object representing that type in python. In the case of list
        is the type of the items.
        e.g.:
            `name = List[int]` -> `(int, True)`
            `name = bool` -> `(bool, False)`
        For `data_type=list`, the returned value is (None, True).
    """
    if data_type not in __supported_types:
        raise ValueError('Unrecognized data type: {}'.format(data_type))
    is_list = False
    if issubclass(data_type, List):
        is_list = True
        data_type = data_type.__args__[0]
    elif data_type is list:
        is_list = True
        data_type = None
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
        - `list` or `List`

    `types = None` works in the same way as:
        `(int, float, bool, list, str)`
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
            if type_obj is None:
                return True
            if all(isinstance(x, type_obj) for x in value):
                return True
        else:
            if isinstance(value, type_obj):
                return True
    return False


def coerce_to_bool(x: str):
    """Convert string to bool value."""
    if x.lower() in ('true', 'yes', 'on', '1', 'false', 'no', 'off', '0'):
        return x.lower() in ('true', 'yes', 'on', '1')
    raise ValueError()


def coerce_to_str(x: str):
    """Strip outer quotes if we have them."""
    if x.startswith("'") and x.endswith('"'):
        return x[1:-1]
    elif x.startswith('"') and x.endswith('"'):
        return x[1:-1]
    else:
        return x


__coercion_rules = {
    str: coerce_to_str,
    bool: coerce_to_bool,
    int: int,
    float: float,
}


def coerce_scalar(x: str, types=None):
    """
    Convert string to int, flot, bool, str with the above conversion rules.

    If types is not `None`, only those conversions are tried.
    If not, all the possible convertions are tried in order.

    :param x: string to be converted.
    :param type_obj: should be `int`, `float`, `bool`, `str`.
        It can also be an iterable combining the above types, or `None`.
    """
    conversions_to_try = types
    if conversions_to_try is None:
        conversions_to_try = __scalar_types
    elif not isinstance(conversions_to_try, Iterable):
        conversions_to_try = [conversions_to_try]
    for t in conversions_to_try:
        try:
            return __coercion_rules[t](x)
        except ValueError:
            pass
    raise ValueError('Not conversion is possible')


def coerce_list(x: List[str], types=None):
    """Coerce each member of the list using `coerce_scalar` function."""
    return [coerce_scalar(i, types) for i in x]


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
        - `list` or `List`

    `types = None` works in the same way as:
        `(int, float, bool, list)`
    """
    if types is None:
        types = __types_for_guess
    elif not isinstance(types, Iterable):
        types = [types]

    value_is_list = isinstance(value, list)

    for x in types:
        type_obj, type_is_list = extract_type(x)
        if type_is_list != value_is_list:
            continue
        if type_is_list:
            try:
                return coerce_list(value, type_obj)
            except ValueError:
                pass
        else:
            try:
                return coerce_scalar(value, type_obj)
            except ValueError:
                pass
    raise ValueError(
        'Can not convert value {} to one of the types in {}'.format(
            value, types
        )
    )
