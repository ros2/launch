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

"""Module with utility for normalizing parameters to a node."""

from collections.abc import Iterable
from collections.abc import Mapping
from collections.abc import Sequence
import pathlib
from typing import cast
from typing import List  # noqa
from typing import Optional
from typing import Sequence as SequenceTypeHint
from typing import Tuple  # noqa
from typing import Union  # noqa

from launch.some_substitutions_type import SomeSubstitutionsType
from launch.some_substitutions_type import SomeSubstitutionsType_types_tuple
from launch.substitution import Substitution
from launch.substitutions import TextSubstitution
from launch.utilities import ensure_argument_type
from launch.utilities import normalize_to_list_of_substitutions

from ..parameters_type import ParameterFile
from ..parameters_type import Parameters
from ..parameters_type import ParametersDict
from ..parameters_type import ParameterValue
from ..parameters_type import SomeParameters
from ..parameters_type import SomeParametersDict
from ..parameters_type import SomeParameterValue


def _normalize_parameter_array_value(value: SomeParameterValue) -> ParameterValue:
    """Normalize substitutions while preserving the type if it's not a substitution."""
    if not isinstance(value, Sequence):
        raise TypeError('Value {} must be a sequence'.format(repr(value)))

    # Figure out what type the list should be
    target_type = None  # type: Optional[type]
    for subvalue in value:
        allowed_subtypes = (float, int, str, bool) + SomeSubstitutionsType_types_tuple
        ensure_argument_type(subvalue, allowed_subtypes, 'subvalue')

        if isinstance(subvalue, Substitution):
            subtype = Substitution  # type: type
        else:
            subtype = type(subvalue)

        if target_type is None:
            target_type = subtype

        if subtype == float and target_type == int:
            # If any value is a float, convert all integers to floats
            target_type = float
        elif subtype == int and target_type == float:
            # If any value is a float, convert all integers to floats
            pass
        elif subtype == str and target_type == Substitution:
            # If any value is a single substitution then result is a single string
            target_type = Substitution
        elif subtype == Substitution and target_type == str:
            # If any value is a single substitution then result is a single string
            target_type = Substitution
        elif subtype != target_type:
            # If types don't match, assume list of strings
            target_type = str

    if target_type is None:
        # No clue what an empty list's type should be
        return []
    elif target_type == Substitution:
        # Keep the list of substitutions together to form a single string
        return tuple(normalize_to_list_of_substitutions(cast(SomeSubstitutionsType, value)))

    if target_type == float:
        return tuple(float(s) for s in value)
    elif target_type == int:
        return tuple(int(s) for s in value)
    elif target_type == bool:
        return tuple(bool(s) for s in value)
    else:
        output_value = []  # type: List[Tuple[Substitution, ...]]
        for subvalue in value:
            if not isinstance(subvalue, Iterable) and not isinstance(subvalue, Substitution):
                # Convert simple types to strings
                subvalue = str(subvalue)
            # Make everything a substitution
            output_value.append(tuple(normalize_to_list_of_substitutions(subvalue)))
        return tuple(output_value)


def normalize_parameter_dict(
    parameters: SomeParametersDict, *,
    _prefix: Optional[SequenceTypeHint[Substitution]] = None
) -> ParametersDict:
    """
    Normalize types used to store parameters in a dictionary.

    The parameters are passed as a dictionary that specifies parameter rules.
    Keys of the dictionary can be strings, a Substitution, or an iterable of Substitution.
    A normalized keys will be a tuple of substitutions.
    Values in the dictionary can be strings, integers, floats, substututions, lists of
    the previous types, or another dictionary with the same properties.

    Normalized values that were lists will have all subvalues converted to the same type.
    If all subvalues are int or float, then the normalized subvalues will all be float.
    If the subvalues otherwise do not all have the same type, then the normalized subvalues
    will be lists of Substitution that will result in string parameters.

    Values that are a list of strings will become a list of strings when normalized and evaluated.
    Values that are a list of :class:`Substitution` will become a single string.
    To make a list of strings from substitutions, each item in the list must be a list or tuple.

    Normalized values that contained nested dictionaries will be collapsed into a single
    layer with parameter names concatenated with the parameter namespace separator ".".

    :param parameters: Parameters to be normalized
    :param _prefix: internal parameter used for flatening recursive dictionaries
    :return: Normalized parameters
    """
    if not isinstance(parameters, Mapping):
        raise TypeError('expected dict')

    normalized = {}  # type: ParametersDict
    for name, value in parameters.items():
        # First make name a list of substitutions
        name = normalize_to_list_of_substitutions(name)
        if _prefix:
            # Prefix name if there is a recursive dictionary
            # weird looking logic to combine into one list to appease mypy
            combined = [e for e in _prefix]
            combined.append(TextSubstitution(text='.'))
            combined.extend(name)
            name = combined

        # Normalize the value next
        if isinstance(value, Mapping):
            # Flatten recursive dictionaries
            sub_dict = normalize_parameter_dict(value, _prefix=name)
            normalized.update(sub_dict)
        elif isinstance(value, str) or isinstance(value, Substitution):
            normalized[tuple(name)] = tuple(normalize_to_list_of_substitutions(value))
        elif isinstance(value, float) or isinstance(value, bool) or isinstance(value, int):
            # Keep some types as is
            normalized[tuple(name)] = value
        elif isinstance(value, bytes):
            # Keep bytes as is
            normalized[tuple(name)] = value
        elif isinstance(value, Sequence):
            # try to make the parameter types uniform
            normalized[tuple(name)] = _normalize_parameter_array_value(value)
        else:
            raise TypeError('Unexpected type for parameter value {}'.format(repr(value)))
    return normalized


def normalize_parameters(parameters: SomeParameters) -> Parameters:
    """
    Normalize the types used to store parameters to substitution types.

    The passed parameters must be an iterable where each element is
    a path to a yaml file or a dict.
    The normalized parameters will have all paths converted to a list of :class:`Substitution`,
    and dictionaries normalized using :meth:`normalize_parameter_dict`.
    """
    if isinstance(parameters, str) or not isinstance(parameters, Sequence):
        raise TypeError('Expecting list of parameters, got {}'.format(parameters))

    normalized_params = []  # type: List[Union[ParameterFile, ParametersDict]]
    for param in parameters:
        if isinstance(param, Mapping):
            normalized_params.append(normalize_parameter_dict(param))
        else:
            # It's a path, normalize to a list of substitutions
            if isinstance(param, pathlib.Path):
                param = str(param)
            ensure_argument_type(param, SomeSubstitutionsType_types_tuple, 'parameters')
            normalized_params.append(tuple(normalize_to_list_of_substitutions(param)))
    return tuple(normalized_params)
