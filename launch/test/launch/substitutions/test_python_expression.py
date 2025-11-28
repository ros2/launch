# Copyright 2022 Open Source Robotics Foundation, Inc.
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

"""Tests for the PythonExpression substitution class."""

from launch import LaunchContext
from launch.substitutions import PythonExpression
from launch.substitutions import SubstitutionFailure

import pytest


def test_python_substitution_missing_module():
    """Check that evaluation fails if we do not pass a needed module (sys)."""
    lc = LaunchContext()
    expr = 'sys.getrefcount(str("hello world!"))'

    subst = PythonExpression([expr])

    # Should raise a NameError since it doesn't see the sys module
    with pytest.raises(NameError):
        subst.perform(lc)

    # Test the describe() method
    assert subst.describe() == "PythonExpr('sys.getrefcount(str(\"hello world!\"))', ['math'])"


def test_python_substitution_no_module():
    """Check that PythonExpression has the math module by default."""
    lc = LaunchContext()
    expr = 'math.ceil(1.6)'

    subst = PythonExpression([expr])
    result = subst.perform(lc)

    assert result == '2'

    # Test the describe() method
    assert subst.describe() == "PythonExpr('math.ceil(1.6)', ['math'])"


def test_python_substitution_implicit_math():
    """Check that PythonExpression will accept math definitions implicitly."""
    lc = LaunchContext()
    expr = 'ceil(1.6)'

    subst = PythonExpression([expr])
    result = subst.perform(lc)

    assert result == '2'

    # Test the describe() method
    assert subst.describe() == "PythonExpr('ceil(1.6)', ['math'])"


def test_python_substitution_empty_module_list():
    """Case where user provides empty module list."""
    lc = LaunchContext()
    expr = 'math.ceil(1.6)'

    subst = PythonExpression([expr], [])

    # Should raise a NameError since it doesn't have the math module
    with pytest.raises(NameError):
        subst.perform(lc)

    # Test the describe() method
    assert subst.describe() == "PythonExpr('math.ceil(1.6)', [])"


def test_python_substitution_one_module():
    """Evaluation while passing one module."""
    lc = LaunchContext()
    expr = 'sys.getrefcount(str("hello world!"))'

    subst = PythonExpression([expr], ['sys'])
    try:
        result = subst.perform(lc)
    except SubstitutionFailure:
        pytest.fail('Failed to evaluate PythonExpression containing sys module.')

    # A refcount should be some positive number
    assert int(result) > 0

    # Test the describe() method
    assert subst.describe() == "PythonExpr('sys.getrefcount(str(\"hello world!\"))', ['sys'])"


def test_python_substitution_two_modules():
    """Evaluation while passing two modules."""
    lc = LaunchContext()
    expr = 'math.isfinite(sys.getrefcount(str("hello world!")))'

    subst = PythonExpression([expr], ['sys', 'math'])
    try:
        result = subst.perform(lc)
    except SubstitutionFailure:
        pytest.fail('Failed to evaluate PythonExpression containing sys module.')

    # The expression should evaluate to True - the refcount is finite
    assert result

    # Test the describe() method
    assert subst.describe() ==\
        "PythonExpr('math.isfinite(sys.getrefcount(str(\"hello world!\")))', ['sys', 'math'])"
