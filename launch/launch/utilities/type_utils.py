# Copyright 2019-2020 Open Source Robotics Foundation, Inc.
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

from typing import List
from typing import Optional
from typing import Text
from typing import Tuple
from typing import Union

import yaml

# Literal is available from Python 3.8
# Delete this when support for Python 3.7 is dropped
try:
    from typing import Literal
except ImportError:
    from typing import Any
    Literal = None

__ScalarTypesTuple = (
    int, float, bool, str
)

__ListTypesTuple = tuple(
    List[x] for x in __ScalarTypesTuple
)

__AllowedTypesTuple = __ScalarTypesTuple + __ListTypesTuple

"""
Type annotation that can be used to specify the allowed scalar types.

The allowed types are:
- int
- str
- bool
- float
"""
ScalarTypes = Literal[__ScalarTypesTuple] if Literal is not None else Any
"""
Type annotation that can be used to specify the allowed list types.

The allowed types are:
- List[int]
- List[str]
- List[bool]
- List[float]
"""
ListTypes = Literal[__ListTypesTuple] if Literal is not None else Any
"""
Type annotation that can be used to specify any of the allowed types.

The allowed types are:
- int
- str
- bool
- float
- List[int]
- List[str]
- List[bool]
- List[float]
"""
AllowedTypes = Literal[__AllowedTypesTuple] if Literal is not None else Any

"""
Type annotation that can be used to specify any of the allowed scalar values.

i.e.: objects that are instances of `ScalarTypes`.
"""
ScalarValues = Union[__ScalarTypesTuple]
"""
Type annotation that can be used to specify any of the allowed list values.

i.e.: objects that are instances of `ListTypes`.
"""
ListValues = Union[__ListTypesTuple]
"""
Type annotation that can be used to specify any of the allowed values.

i.e.: objects that are instances of `AllowedTypes`.
"""
AllowedValues = Union[__AllowedTypesTuple]

"""Type annotation that can be used to specify any Scalar or a substitution."""
SomeScalarValuesType = Union[__AllowedTypesTuple, SomeSubstitutionsType]
"""Type annotation that can be used to specify any List or a substitution."""
SomeListValuesType = Union[tuple(List[Union[x, SomeSubstitutionsType]] for x in __ScalarTypesTuple)]
"""Type annotation that can be used to specify Value or Substitutions."""
SomeAllowedValuesType = Union[SomeScalarValuesType, SomeListValuesType]



def check_is_typing_list(data_type: AllowedTypes) -> bool:
    """Check if `data_type` is based on a `typing.List`."""
    return hasattr(data_type, '__origin__') and \
        data_type.__origin__ in (list, List)  # On Linux/Mac is List, on Windows is list.


def check_valid_scalar_type(data_type: AllowedTypes) -> bool:
    """Check if `data_type` is a valid scalar type."""
    return data_type in __ScalarTypesTuple


def extract_type(data_type: AllowedTypes) -> Tuple[ScalarTypes, bool]:
    """
    Extract type information from type object.

    :param data_type: It can be:
        - a scalar type i.e. `str`, `int`, `float`, `bool`;
        - a uniform list i.e `List[str]`, `List[int]`, `List[float]`, `List[bool]`;

    :returns: a tuple (type_obj, is_list).
        is_list is `True` for the supported list types, if not is `False`.
        type_obj is the object representing that type in python. In the case of list
        is the type of the items.
        e.g.:
            `data_type = List[int]` -> `(int, True)`
            `data_type = bool` -> `(bool, False)`
    """
    is_list = False
    if check_is_typing_list(data_type):
        is_list = True
        data_type = data_type.__args__[0]
    if check_valid_scalar_type(data_type) is False:
        raise ValueError('Unrecognized data type: {}'.format(data_type))
    return (data_type, is_list)


def check_is_instance_of_valid_type(value: AllowedValues):
    """Check if value is an instance of an allowed type."""
    if isinstance(value, list):
        if not value:
            return True  # Accept empty lists.
        member_type = type(value[0])
        return (
            all(isinstance(x, member_type) for x in value[1:]) and
            member_type in __ScalarTypesTuple
        )
    return isinstance(value, __ScalarTypesTuple)


def check_type(value: AllowedValues, data_type: Optional[AllowedTypes]) -> bool:
    """
    Check if `value` is of `type`.

    The allowed types are:
        - a scalar type i.e. `str`, `int`, `float`, `bool`;
        - a uniform list i.e `List[str]`, `List[int]`, `List[float]`, `List[bool]`;
    """
    if data_type is None:
        return check_is_instance_of_valid_type(value)
    type_obj, is_list = extract_type(data_type)
    if not is_list:
        return isinstance(value, type_obj)
    if not isinstance(value, list):
        return False
    return all(isinstance(x, type_obj) for x in value)


def coerce_to_type(
    value: Text,
    data_type: Optional[AllowedTypes] = None
) -> AllowedValues:
    """
    Try to convert `value` to the type specified in `data_type`.

    If not raise `ValueError`.

    The allowed types are:
        - a scalar type i.e. `str`, `int`, `float`, `bool`;
        - a uniform list i.e `List[str]`, `List[int]`, `List[float]`, `List[bool]`;
        - `None`: try to use yaml convertion rules, and checks if the output is
          a scalar or an uniform list.

    The coercion order for scalars is always: `int`, `float`, `bool`, `str`.
    """
    def convert_as_yaml(value, error_msg):
        try:
            output = yaml.safe_load(value)
        except Exception as err:
            raise ValueError(f'{error_msg}: yaml.safe_load() failed\n{err}')

        if not check_is_instance_of_valid_type(output):
            raise ValueError(
                f'{error_msg}: output type is not allowed, got {type(output)}'
            )
        return output

    if data_type is None:
        # No type specified, return best conversion.
        return convert_as_yaml(value, f"Failed to convert '{value}' using yaml rules")

    type_obj, is_list = extract_type(data_type)

    if is_list:
        output = convert_as_yaml(
            value, f"Cannot convert value '{value}' to a list of '{type_obj}'")
        if not isinstance(output, list):
            raise ValueError(f"Cannot convert value '{value}' to a list of '{type_obj}'")
        if not all(isinstance(x, type_obj) for x in output):
            raise ValueError(
                f"Cannot convert value '{value}' to a list of '{type_obj}', got {output}")
        return output

    if type_obj is str:
        return value
    if type_obj in (int, float):
        return type_obj(value)

    assert bool == type_obj, 'This error should not happen, please open an issue'
    output = convert_as_yaml(value, f"Failed to convert '{value}' to '{type_obj}'")
    if isinstance(output, type_obj):
        return output
    raise ValueError(f"Cannot convert value '{value}' to '{type_obj}'")


def coerce_list(x: List[str], data_type: Optional[ScalarTypes] = None) -> ListValues:
    """Coerce each member of the list using `coerce_scalar` function."""
    return [coerce_to_type(i, data_type) for i in x]


def get_typed_value(
    value: Union[Text, List[Text]],
    data_type: Optional[AllowedTypes]
) -> AllowedValues:
    """
    Try to convert `value` to the type specified in `data_type`.

    If not raise `AttributeError`.

    The allowed types are:
        - a scalar type i.e. `str`, `int`, `float`, `bool`;
        - a uniform list i.e `List[str]`, `List[int]`, `List[float]`, `List[bool]`, List;

    `types = None` works in the same way as:
        `Union[int, float, bool, list, str]`

    The coercion order for scalars is always: `int`, `float`, `bool`, `str`.
    """
    if isinstance(value, list):
        if data_type is not None:
            data_type, is_list = extract_type(data_type)
            if not is_list:
                raise TypeError(
                    f"Cannot convert input '{value}' of type '{type(value)}' to"
                    f" '{data_type}'"
                )
        value = coerce_list(value, data_type)
    else:
        value = coerce_to_type(value, data_type)
    return value
