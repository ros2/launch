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

from launch.utilities.type_utils import check_is_instance_of_valid_type
from launch.utilities.type_utils import check_type
from launch.utilities.type_utils import coerce_to_type

import pytest


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


def test_check_type():
    assert check_type(1, int)
    assert check_type(1., float)
    assert check_type('asd', str)
    assert check_type(True, bool)

    assert check_type([1, 2], List[int])
    assert check_type([1., 2.], List[float])
    assert check_type(['asd', 'bsd'], List[str])
    assert check_type([True, False], List[bool])

    assert not check_type(1., int)
    assert not check_type(1, float)
    assert not check_type(True, str)
    assert not check_type('asd', bool)

    assert not check_type([1, 2.], List[int])
    assert not check_type([1, 2.], List[float])
    assert not check_type([True, False], List[str])
    assert not check_type(['True', 'False'], List[bool])


def test_check_type_fails():
    with pytest.raises(ValueError):
        check_type([True, False], list)
    with pytest.raises(ValueError):
        check_type([True, False], Union[int, str])


def test_check_is_instance_of_valid_type():
    assert check_is_instance_of_valid_type(1)
    assert check_is_instance_of_valid_type(1.)
    assert check_is_instance_of_valid_type('asd')
    assert check_is_instance_of_valid_type(True)

    assert check_is_instance_of_valid_type([1, 2])
    assert check_is_instance_of_valid_type([1., 2.])
    assert check_is_instance_of_valid_type(['asd', 'bsd'])
    assert check_is_instance_of_valid_type([True, False])

    assert not check_is_instance_of_valid_type([1, '2'])
    assert not check_is_instance_of_valid_type(object)
    assert not check_is_instance_of_valid_type(test_check_is_instance_of_valid_type)
    assert not check_is_instance_of_valid_type({'key': 'value'})
