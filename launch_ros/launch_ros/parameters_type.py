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

"""Module for ROS parameter types."""

import pathlib

from typing import Any
from typing import Dict
from typing import Mapping
from typing import Sequence
from typing import Union

from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitution import Substitution


# Parameter value types used below
_SingleValueType = Union[str, int, float, bool]
_MultiValueType = Union[
    Sequence[str], Sequence[int], Sequence[float], Sequence[bool], bytes]

SomeParameterFile = Union[SomeSubstitutionsType, pathlib.Path]
SomeParameterName = Sequence[Union[Substitution, str]]
SomeParameterValue = Union[SomeSubstitutionsType,
                           Sequence[SomeSubstitutionsType],
                           _SingleValueType,
                           _MultiValueType]

# TODO(sloretz) Recursive type when mypy supports them python/mypy#731
_SomeParametersDict = Mapping[SomeParameterName, Any]
SomeParametersDict = Mapping[SomeParameterName, Union[SomeParameterValue, _SomeParametersDict]]

# Paths to yaml files with parameters, or dictionaries of parameters, or pairs of
# parameter names and values
SomeParameters = Sequence[Union[SomeParameterFile, Mapping[SomeParameterName, SomeParameterValue]]]

ParameterFile = Sequence[Substitution]
ParameterName = Sequence[Substitution]
ParameterValue = Union[Sequence[Substitution],
                       Sequence[Sequence[Substitution]],
                       _SingleValueType,
                       _MultiValueType]

# Normalized (flattened to avoid having a recursive type) parameter dict
ParametersDict = Dict[ParameterName, ParameterValue]

# Normalized parameters
Parameters = Sequence[Union[ParameterFile, ParametersDict]]

EvaluatedParameterValue = Union[_SingleValueType, _MultiValueType]
# Evaluated parameters: filenames or dictionary after substitutions have been evaluated
EvaluatedParameters = Sequence[Union[pathlib.Path, Dict[str, EvaluatedParameterValue]]]
