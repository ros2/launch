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

import collections.abc
from typing import Any
from typing import cast
from typing import List
from typing import Optional
from typing import Sequence
from typing import Set
from typing import Text
from typing import Tuple
from typing import Type
from typing import Union

import yaml

from .normalize_to_list_of_substitutions_impl import normalize_to_list_of_substitutions
from .perform_substitutions_impl import perform_substitutions
from ..launch_context import LaunchContext
from ..some_substitutions_type import SomeSubstitutionsType
from ..substitution import Substitution

ScalarTypesTuple = (
    int, float, bool, str
)

ListTypesTuple = (
    List[int], List[float], List[bool], List[str]
)

AllowedTypesTuple = ScalarTypesTuple + ListTypesTuple

"""Allowed scalar types."""
ScalarTypesType = Type[Union[int, float, bool, str]]
"""Allowed uniform list types."""
ListTypesType = Type[Union[
    List[int], List[float], List[bool], List[str]
]]
"""Allowed types."""
AllowedTypesType = Union[ScalarTypesType, ListTypesType]

"""Allowed scalar values."""
ScalarValueType = Union[int, float, bool, str]
"""Allowed uniform list values."""
ListValueType = Union[List[int], List[float], List[bool], List[str]]
"""Allowed uniform sequence values."""
SequenceValueType = Union[Sequence[int], Sequence[float], Sequence[bool], Sequence[str]]
"""Allowed values."""
AllowedValueType = Union[ScalarValueType, SequenceValueType]

"""Substitution to a scalar type or a scalar type."""
SomeScalarType = Union[SomeSubstitutionsType, ScalarValueType]
"""Substitution that can be performed to a uniform list or a uniform list."""
SomeSequenceType = Union[(
    Sequence[Union[int, SomeSubstitutionsType]],
    Sequence[Union[float, SomeSubstitutionsType]],
    Sequence[Union[bool, SomeSubstitutionsType]],
    Sequence[Union[str, SomeSubstitutionsType]],
)]
"""Union of SomeScalarType and SomeSequenceType."""
SomeValueType = Union[SomeScalarType, SomeSequenceType]

NormalizedScalarType = Union[List[Substitution], ScalarValueType]
NormalizedSequenceType = Union[
    List[Union[int, List[Substitution]]],
    List[Union[float, List[Substitution]]],
    List[Union[bool, List[Substitution]]],
    List[Union[str, List[Substitution]]],
]
NormalizedValueType = Union[NormalizedScalarType, NormalizedSequenceType]


def check_is_typing_list(data_type: Any) -> bool:
    """Check if `data_type` is based on a `typing.List`."""
    return data_type is List or (
        hasattr(data_type, '__origin__') and
        hasattr(data_type, '__args__') and
        data_type.__origin__ in  # type: ignore
        (list, List) and  # On Linux/Mac is List, on Windows is list.
        len(data_type.__args__) > 0 and
        data_type is List[data_type.__args__[0]]  # type: ignore
    )


def check_valid_scalar_type(data_type: AllowedTypesType) -> bool:
    """Check if `data_type` is a valid scalar type."""
    return data_type in ScalarTypesTuple


def extract_type(data_type: AllowedTypesType) -> Tuple[ScalarTypesType, bool]:
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
    scalar_type: ScalarTypesType = cast(ScalarTypesType, data_type)
    if check_is_typing_list(data_type):
        is_list = True
        scalar_type = data_type.__args__[0]  # type: ignore
    if check_valid_scalar_type(scalar_type) is False:
        raise ValueError(f'Unrecognized data type: {data_type}')
    return (scalar_type, is_list)


def check_is_instance_of_valid_type(value: Any) -> bool:
    """Check if value is an instance of an allowed type."""
    if isinstance(value, list):
        if not value:
            return True  # Accept empty lists.
        member_type = type(value[0])
        return (
            all(isinstance(x, member_type) for x in value[1:]) and
            member_type in ScalarTypesTuple
        )
    return isinstance(value, ScalarTypesTuple)


def check_type(value: Any, data_type: Optional[AllowedTypesType]) -> bool:
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
    data_type: Optional[AllowedTypesType] = None
) -> AllowedValueType:
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


def coerce_list(value: List[str], data_type: Optional[ScalarTypesType] = None) -> ListValueType:
    """Coerce each member of the list using `coerce_scalar` function."""
    output = [coerce_to_type(i, data_type) for i in value]
    if not check_is_instance_of_valid_type(output):
        raise ValueError(f"cannot convert value to {data_type}. Got value='{value}'")
    return cast(ListValueType, output)


def get_typed_value(
    value: Union[Text, List[Text]],
    data_type: Optional[AllowedTypesType]
) -> AllowedValueType:
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
        output: AllowedValueType = coerce_list(value, data_type)
    else:
        output = coerce_to_type(value, data_type)
    return output


def is_substitution(x):
    # is some substitution
    return (
        isinstance(x, Substitution) or
        (
            isinstance(x, collections.abc.Iterable) and
            len(x) > 0 and
            all(isinstance(item, (Substitution, str)) for item in x) and
            any(isinstance(item, Substitution) for item in x)
        )
    )


def normalize_typed_substitution(
    value: SomeValueType, data_type: Optional[AllowedTypesType]
) -> NormalizedValueType:
    # Resolve scalar types immediately
    if isinstance(value, ScalarTypesTuple):
        if not check_type(value, data_type):
            raise TypeError(f"value='{value}' is not an instance of {data_type}")
        return value
    # Resolve substitutions and list of substitutions immediately
    if is_substitution(value):
        return normalize_to_list_of_substitutions(cast(SomeSubstitutionsType, value))
    # For the other cases, the input must be an iterable
    if not isinstance(value, collections.abc.Iterable):
        raise TypeError(
            'value should be either a scalar, a substitutions,'
            ' or a mixed list of scalars and substitutions. '
            f"Got 'value={value}' of type '{type(value)}'. "
        )
    # Collect the types of the items of the list
    types_in_list: Set[Optional[Type[Union[str, int, float, bool, Substitution]]]] = set()
    for x in value:
        if isinstance(x, ScalarTypesTuple):
            types_in_list.add(type(x))
        elif is_substitution(x):
            types_in_list.add(Substitution)
        else:
            types_in_list.add(None)
    if None in types_in_list:
        raise TypeError(
            'value is a list, and one of the items is not a scalar, a Substitution '
            f"or a list of substitutions. Got value='{value}'"
        )
    # Extract expected type information
    is_list = True
    if data_type is not None:
        data_type, is_list = extract_type(data_type)
    # Must be expecting a list
    if not is_list:
        raise TypeError(
            'The provided value resolves to a list, though the required type is a scalar. '
            f"Got value='{value}', data_type='{data_type}'."
        )

    # Normalize each specific uniform list input
    err_msg = (
        "Got a list of '{}'"
        f", expected a list of '{data_type}'. value='{value}'"
    )
    if types_in_list.issubset({str, Substitution}):
        # list of mixed strings and substitutions
        if data_type not in (None, str):
            raise TypeError(err_msg.format(str))
        return cast(List[Union[List[Substitution], str]], [
            normalize_to_list_of_substitutions(cast(SomeSubstitutionsType, x)) for x in value
        ])
    if types_in_list.issubset({bool, Substitution}):
        # list of booleans and substitutions
        if data_type not in (None, bool):
            raise TypeError(err_msg.format(bool))
        return cast(List[Union[List[Substitution], bool]], [
            normalize_to_list_of_substitutions(cast(SomeSubstitutionsType, x))
            if is_substitution(x) else x for x in value
        ])
    if types_in_list.issubset({int, Substitution}):
        # list of ints and substitutions
        if data_type not in (None, int):
            raise TypeError(err_msg.format(int))
        return cast(List[Union[List[Substitution], int]], [
            normalize_to_list_of_substitutions(cast(SomeSubstitutionsType, x))
            if is_substitution(x) else x for x in value
        ])
    if types_in_list.issubset({int, float, Substitution}):
        # list of floats and substitutions
        if data_type not in (None, float):
            raise TypeError(err_msg.format(int))
        return cast(List[Union[List[Substitution], float]], [
            normalize_to_list_of_substitutions(cast(SomeSubstitutionsType, x))
            if is_substitution(x) else float(cast(Union[int, float], x)) for x in value
        ])
    # Invalid input
    raise TypeError(
        "Input value is not an acceptable 'SomeValueType'."
        f"Got value='{value}'"
    )


def is_normalized_substitution(x):
    return (
        isinstance(x, list) and
        len(x) > 0 and
        all(isinstance(item, Substitution) for item in x)
    )


def perform_typed_substitution(
    context: LaunchContext,
    value: NormalizedValueType,
    data_type: Optional[AllowedTypesType]
) -> AllowedValueType:
    if isinstance(value, ScalarTypesTuple):
        return value
    elif is_normalized_substitution(value):
        return coerce_to_type(
            perform_substitutions(context, cast(List[Substitution], value)),
            data_type
        )
    elif isinstance(value, list):
        scalar_type: Optional[ScalarTypesType] = None
        if data_type is not None:
            scalar_type, is_list = extract_type(data_type)
            if not is_list:
                raise ValueError(
                    'The input value is a list, cannot convert it to a scalar. '
                    f"Got value='{value}', expected type {scalar_type}."
                )
        output = [
            coerce_to_type(
                perform_substitutions(context, cast(List[Substitution], x)), scalar_type)
            if is_normalized_substitution(x) else x for x in value
        ]
        check_type(output, data_type)
        return cast(ListValueType, output)
    # Invalid input
    raise TypeError(
        "Input value is not an acceptable 'NormalizedValueType'."
        f"Got value='{value}'"
    )
