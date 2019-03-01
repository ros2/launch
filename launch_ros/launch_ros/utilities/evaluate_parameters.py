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

"""Module with utility for evaluating parametesr in a launch context."""


from collections.abc import Mapping
from collections.abc import Sequence
import pathlib
from typing import Dict
from typing import List
from typing import Union

from launch.launch_context import LaunchContext
from launch.substitution import Substitution
from launch.utilities import ensure_argument_type
from launch.utilities import perform_substitutions

from ..parameters_type import EvaluatedParameters
from ..parameters_type import EvaluatedParameterValue  # noqa
from ..parameters_type import Parameters


def evaluate_parameters(context: LaunchContext, parameters: Parameters) -> EvaluatedParameters:
    """
    Evaluate substitutions to produce paths and name/value pairs.

    The parameters must have been normalized with normalize_parameters() prior to calling this.

    :param parameters: normalized parameters
    :returns: values after evaluating lists of substitutions
    """
    output_params = []  # type: List[Union[pathlib.Path, Dict[str, EvaluatedParameterValue]]]
    for param in parameters:
        # If it's a list of substitutions then evaluate them to a string and return a pathlib.Path
        if isinstance(param, tuple) and len(param) and isinstance(param[0], Substitution):
            # Evaluate a list of Substitution to a file path
            output_params.append(pathlib.Path(perform_substitutions(context, list(param))))
        elif isinstance(param, Mapping):
            # It's a list of name/value pairs
            output_dict = {}  # type: Dict[str, EvaluatedParameterValue]
            for name, value in param.items():
                if not isinstance(name, tuple):
                    raise TypeError('Expecting tuple of substitutions got {}'.format(repr(name)))
                name = perform_substitutions(context, list(name))

                if isinstance(value, tuple) and len(value):
                    if isinstance(value[0], Substitution):
                        # Value is a list of substitutions, so perform them to make a string
                        value = perform_substitutions(context, list(value))
                    elif isinstance(value[0], Sequence):
                        # Value is an array of a list of substitutions
                        output_subvalue = []  # List[str]
                        for subvalue in value:
                            output_subvalue.append(perform_substitutions(context, list(subvalue)))
                        value = tuple(output_subvalue)
                    else:
                        # Value is an array of the same type, so nothing to evaluate.
                        output_value = []
                        target_type = type(value[0])
                        for i, subvalue in enumerate(value):
                            output_value.append(target_type(subvalue))
                        value = tuple(output_value)
                else:
                    # Value is a singular type, so nothing to evaluate
                    ensure_argument_type(value, (float, int, str, bool, bytes), 'value')
                output_dict[name] = value
            output_params.append(output_dict)
    return tuple(output_params)
