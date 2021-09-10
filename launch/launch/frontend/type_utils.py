# Copyright 2020 Open Source Robotics Foundation, Inc.
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

"""Extra type utils for launch frontend implementations."""

from typing import Any
from typing import List
from typing import Text
from typing import Tuple
from typing import Type
from typing import Union

from .entity import Entity
from ..utilities.type_utils import AllowedTypesType
from ..utilities.type_utils import is_typing_list


def check_is_list_entity(data_type: Union[AllowedTypesType, Type[List[Entity]]]) -> bool:
    """Check if `data_type` is a `typing.List` with elements of `Entity` type or derived."""
    return is_typing_list(data_type) and \
        issubclass(data_type.__args__[0], Entity)  # type: ignore


def get_data_type_from_identifier(type_identifier: Text):
    mapping = {
        'str': str,
        'bool': bool,
        'float': float,
        'int': int,
        'list_of_str': List[str],
        'list_of_bool': List[bool],
        'list_of_float': List[float],
        'list_of_int': List[int],
        'yaml': None,
    }
    if type_identifier not in mapping:
        raise ValueError(f"Got invalid type identifier '{type_identifier}'")
    return mapping[type_identifier]


# The following definitions were moved/refactored into launch.utilities.type_utils since Foxy
# Kept here for backwards compatibility

__ScalarTypesTuple = (
    int, float, bool, str
)

__TypesForGuessTuple = (
    int, float, bool, list, str
)


def check_is_list(data_type: Any) -> bool:
    """Check if `data_type` is based on a `typing.List`."""
    return hasattr(data_type, '__origin__') and \
        data_type.__origin__ in (list, List)  # On Linux/Mac is List, on Windows is list.


def check_is_union(data_type: Any) -> bool:
    """Check if `data_type` is based on a `typing.Union`."""
    return hasattr(data_type, '__origin__') and \
        data_type.__origin__ is Union


def get_tuple_of_types(data_type: Any) -> Tuple:
    """
    Normalize `data_type` to a tuple of types.

    If `data_type` is based on a `typing.Union`, return union types.
    Otherwise, return a `(data_type,)` tuple.
    """
    if check_is_union(data_type):
        return data_type.__args__
    else:
        return (data_type,)


def check_valid_scalar_type(data_type: Any) -> bool:
    """Check if `data_type` is a valid scalar type."""
    return all(data_type in __ScalarTypesTuple for x in get_tuple_of_types(data_type))


def extract_type(data_type: Any) -> Tuple[Any, bool]:
    """
    Extract type information from type object.

    :param data_type: It can be:
        - a scalar type i.e. `str`, `int`, `float`, `bool`;
        - a uniform list i.e `List[str]`, `List[int]`, `List[float]`, `List[bool]`;
        - a non-uniform list of known scalar types e.g. `List[Union[int, str]]`;
        - a non-uniform list of any scalar type i.e. `list` or `List`;
    :returns: a tuple (type_obj, is_list).
        is_list is `True` for the supported list types, if not is `False`.
        type_obj is the object representing that type in python. In the case of list
        is the type of the items.
        e.g.:
            `name = List[int]` -> `(int, True)`
            `name = bool` -> `(bool, False)`
            `name = Union[bool, str]` -> `(Union[bool, str], False)`
            `name = List[Union[bool, str]]` -> `(Union[bool, str], True)`
            `name = List -> `(None, True)`
    """
    is_list = False
    if data_type is list:
        is_list = True
        data_type = None
    elif check_is_list(data_type):
        is_list = True
        data_type = data_type.__args__[0]
    if data_type is not None and check_valid_scalar_type(data_type) is False:
        raise ValueError('Unrecognized data type: {}'.format(data_type))
    return (data_type, is_list)


def check_type(value: Any, data_type: Any) -> bool:
    """
    Check if `value` is of `type`.

    The allowed types are:
        - a scalar type i.e. `str`, `int`, `float`, `bool`;
        - a uniform list i.e `List[str]`, `List[int]`, `List[float]`, `List[bool]`;
        - a non-uniform list of known scalar types e.g. `List[Union[int, str]]`;
        - a non-uniform list of any scalar type i.e. `list` or `List`;
        - a `Union` of any of the above.
    `types = None` works in the same way as:
        `Union[int, float, bool, list, str]`
    """
    def check_scalar_type(value, data_type):
        data_type = get_tuple_of_types(data_type)
        return isinstance(value, data_type)
    types = __TypesForGuessTuple
    if data_type is not None:
        data_type = get_tuple_of_types(data_type)
    for x in types:
        type_obj, is_list = extract_type(x)
        if is_list:
            if not isinstance(value, list) or not value:
                continue
            if type_obj is None:
                return True
            if all(check_scalar_type(x, type_obj) for x in value):
                return True
        else:
            if check_scalar_type(value, type_obj):
                return True
    return False


def coerce_to_bool(x: str) -> bool:
    """Convert string to bool value."""
    if x.lower() in ('true', 'yes', 'on', '1', 'false', 'no', 'off', '0'):
        return x.lower() in ('true', 'yes', 'on', '1')
    raise ValueError()


def coerce_to_str(x: str) -> str:
    """Strip outer quotes if we have them."""
    if x.startswith("'") and x.endswith("'"):
        return x[1:-1]
    elif x.startswith('"') and x.endswith('"'):
        return x[1:-1]
    else:
        return x


def scalar_type_key(data_type: Any) -> int:
    """Get key. Used for sorting the scalar data_types."""
    keys = {
        int: 0,
        float: 1,
        bool: 2,
        str: 3,
    }
    return keys[data_type]


def coerce_scalar(x: str, data_type: Any = None) -> Union[int, str, float, bool]:
    """
    Convert string to int, flot, bool, str with the above conversion rules.

    If data_type is not `None`, only those conversions are tried.
    If not, all the possible convertions are tried.
    The order is always: `int`, `float`, `bool`, `str`.
    :param x: string to be converted.
    :param type_obj: should be `int`, `float`, `bool`, `str`.
        It can also be an iterable combining the above types, or `None`.
    """
    coercion_rules = {
        str: coerce_to_str,
        bool: coerce_to_bool,
        int: int,
        float: float,
    }
    if data_type is None:
        conversions_to_try = __ScalarTypesTuple
    else:
        conversions_to_try = sorted(get_tuple_of_types(data_type), key=scalar_type_key)
        if not set(conversions_to_try).issubset(set(__ScalarTypesTuple)):
            raise ValueError('Unrecognized data type: {}'.format(data_type))
    for t in conversions_to_try:
        try:
            return coercion_rules[t](x)
        except ValueError:
            pass
    raise ValueError('Not conversion is possible')


def coerce_list(x: List[str], data_type: Any = None) -> List[Union[int, str, float, bool]]:
    """Coerce each member of the list using `coerce_scalar` function."""
    return [coerce_scalar(i, data_type) for i in x]


def get_typed_value(
    value: Union[Text, List[Text]],
    data_type: Any
) -> Union[
    List[Union[int, str, float, bool]],
    Union[int, str, float, bool],
]:
    """
    Try to convert `value` to the type specified in `data_type`.

    If not raise `AttributeError`.
    The allowed types are:
        - a scalar type i.e. `str`, `int`, `float`, `bool`;
        - a uniform list i.e `List[str]`, `List[int]`, `List[float]`, `List[bool]`;
        - a non-uniform list of known scalar types e.g. `List[Union[int, str]]`;
        - a non-uniform list of any scalar type i.e. `list` or `List`;
        - a `Union` of any of the above.
    `types = None` works in the same way as:
        `Union[int, float, bool, list, str]`
    The coercion order for scalars is always: `int`, `float`, `bool`, `str`.
    """
    if data_type is None:
        types = __TypesForGuessTuple
    else:
        types = get_tuple_of_types(data_type)

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
