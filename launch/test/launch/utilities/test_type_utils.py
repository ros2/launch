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

"""Test type checking/coercion utils."""

from typing import List
from typing import Union

from launch.utilities.type_utils import coerce_to_type
from launch.utilities.type_utils import extract_type
from launch.utilities.type_utils import is_instance_of
from launch.utilities.type_utils import is_instance_of_valid_type
from launch.utilities.type_utils import is_typing_list
from launch.utilities.type_utils import is_valid_scalar_type

import pytest


def test_is_typing_list():
    assert is_typing_list(List)
    assert is_typing_list(List[int])
    assert is_typing_list(List[float])
    assert is_typing_list(List[Union[int, float]])
    assert not is_typing_list(list)
    assert not is_typing_list(int)
    assert not is_typing_list(Union[int, float])
    assert not is_typing_list(None)


def test_is_valid_scalar_type():
    assert is_valid_scalar_type(int)
    assert is_valid_scalar_type(float)
    assert is_valid_scalar_type(bool)
    assert is_valid_scalar_type(str)
    assert not is_valid_scalar_type(bytes)
    assert not is_valid_scalar_type(None)
    assert not is_valid_scalar_type(List[float])
    assert not is_valid_scalar_type(list)
    assert not is_typing_list(int)


def test_extract_type():
    assert extract_type(int) == (int, False)
    assert extract_type(float) == (float, False)
    assert extract_type(bool) == (bool, False)
    assert extract_type(str) == (str, False)

    assert extract_type(List[int]) == (int, True)
    assert extract_type(List[float]) == (float, True)
    assert extract_type(List[bool]) == (bool, True)
    assert extract_type(List[str]) == (str, True)

    with pytest.raises(ValueError):
        extract_type(List)
    with pytest.raises(ValueError):
        extract_type(bytes)


def test_coercions_using_yaml_rules():
    assert coerce_to_type('asd') == 'asd'
    assert coerce_to_type('tRuE') == 'tRuE'
    assert coerce_to_type("'1'") == '1'
    assert coerce_to_type("'off'") == 'off'

    assert coerce_to_type('1') == 1
    assert coerce_to_type('1000') == 1000

    assert coerce_to_type('1.') == 1.0
    assert coerce_to_type('1000.0') == 1000.
    assert coerce_to_type('0.2') == .2
    assert coerce_to_type('.3') == 0.3

    assert coerce_to_type('on') is True
    assert coerce_to_type('off') is False
    assert coerce_to_type('True') is True

    assert coerce_to_type('[2, 1, 1]') == [2, 1, 1]
    assert coerce_to_type('[.2, .1, .1]') == [.2, .1, .1]
    assert coerce_to_type('[asd, bsd, csd]') == ['asd', 'bsd', 'csd']
    assert coerce_to_type('[on, false, no]') == [True, False, False]


def test_coercions_given_specific_type():
    assert coerce_to_type('asd', data_type=str) == 'asd'
    assert coerce_to_type('tRuE', data_type=str) == 'tRuE'
    assert coerce_to_type("'1'", data_type=str) == "'1'"
    assert coerce_to_type("'off'", data_type=str) == "'off'"
    assert coerce_to_type("''1''", data_type=str) == "''1''"
    assert coerce_to_type('{1}', data_type=str) == '{1}'

    assert coerce_to_type('1', data_type=int) == 1
    assert coerce_to_type('1000', data_type=int) == 1000

    assert coerce_to_type('1', data_type=float) == 1.0
    assert coerce_to_type('1.', data_type=float) == 1.0
    assert coerce_to_type('1000.0', data_type=float) == 1000.
    assert coerce_to_type('0.2', data_type=float) == .2
    assert coerce_to_type('.3', data_type=float) == 0.3

    assert coerce_to_type('on', data_type=bool) is True
    assert coerce_to_type('off', data_type=bool) is False
    assert coerce_to_type('True', data_type=bool) is True

    assert coerce_to_type('[.2, .1, .1]', data_type=List[float]) == [.2, .1, .1]
    assert coerce_to_type('[asd, bsd, csd]', data_type=List[str]) == ['asd', 'bsd', 'csd']
    assert coerce_to_type('[on, false, no]', data_type=List[bool]) == [True, False, False]


def test_coercion_fails():
    with pytest.raises(ValueError):
        coerce_to_type("''1''")
    with pytest.raises(ValueError):
        coerce_to_type('{1}')
    with pytest.raises(ValueError):
        coerce_to_type('[1, 2.0]')
    with pytest.raises(ValueError):
        coerce_to_type('[asd, 2.0]')

    with pytest.raises(ValueError):
        coerce_to_type('1000.5', data_type=int)
    with pytest.raises(ValueError):
        coerce_to_type('Bsd', data_type=int)

    with pytest.raises(ValueError):
        coerce_to_type('Bsd', data_type=float)

    with pytest.raises(ValueError):
        coerce_to_type('Bsd', data_type=bool)
    with pytest.raises(ValueError):
        coerce_to_type('1', data_type=bool)

    with pytest.raises(ValueError):
        coerce_to_type('[1, 2.0]', data_type=List[float])
    with pytest.raises(ValueError):
        coerce_to_type('[asd, 2.0]', data_type=List[float])


def test_is_instance_of():
    assert is_instance_of(1, int)
    assert is_instance_of(1., float)
    assert is_instance_of('asd', str)
    assert is_instance_of(True, bool)

    assert is_instance_of([1, 2], List[int])
    assert is_instance_of([1., 2.], List[float])
    assert is_instance_of(['asd', 'bsd'], List[str])
    assert is_instance_of([True, False], List[bool])

    assert not is_instance_of(1., int)
    assert not is_instance_of(1, float)
    assert not is_instance_of(True, str)
    assert not is_instance_of('asd', bool)

    assert not is_instance_of([1, 2.], List[int])
    assert not is_instance_of([1, 2.], List[float])
    assert not is_instance_of([True, False], List[str])
    assert not is_instance_of(['True', 'False'], List[bool])


def test_is_instance_of_fails():
    with pytest.raises(ValueError):
        is_instance_of([True, False], list)
    with pytest.raises(ValueError):
        is_instance_of([True, False], Union[int, str])


def test_is_instance_of_valid_type():
    assert is_instance_of_valid_type(1)
    assert is_instance_of_valid_type(1.)
    assert is_instance_of_valid_type('asd')
    assert is_instance_of_valid_type(True)

    assert is_instance_of_valid_type([1, 2])
    assert is_instance_of_valid_type([1., 2.])
    assert is_instance_of_valid_type(['asd', 'bsd'])
    assert is_instance_of_valid_type([True, False])

    assert not is_instance_of_valid_type([1, '2'])
    assert not is_instance_of_valid_type(object)
    assert not is_instance_of_valid_type(test_is_instance_of_valid_type)
    assert not is_instance_of_valid_type({'key': 'value'})
