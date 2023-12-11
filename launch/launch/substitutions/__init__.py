# Copyright 2018 Open Source Robotics Foundation, Inc.
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

"""Package for substitutions."""

from .anon_name import AnonName
from .boolean_substitution import AllSubstitution
from .boolean_substitution import AndSubstitution
from .boolean_substitution import AnySubstitution
from .boolean_substitution import NotSubstitution
from .boolean_substitution import OrSubstitution
from .command import Command
from .environment_variable import EnvironmentVariable
from .equals_substitution import EqualsSubstitution
from .file_content import FileContent
from .find_executable import FindExecutable
from .if_else_substitution import IfElseSubstitution
from .launch_configuration import LaunchConfiguration
from .launch_log_dir import LaunchLogDir
from .local_substitution import LocalSubstitution
from .not_equals_substitution import NotEqualsSubstitution
from .path_join_substitution import PathJoinSubstitution
from .python_expression import PythonExpression
from .substitution_failure import SubstitutionFailure
from .text_substitution import TextSubstitution
from .this_launch_file import ThisLaunchFile
from .this_launch_file_dir import ThisLaunchFileDir

__all__ = [
    'AllSubstitution',
    'AndSubstitution',
    'AnySubstitution',
    'AnonName',
    'Command',
    'EqualsSubstitution',
    'EnvironmentVariable',
    'FileContent',
    'FindExecutable',
    'IfElseSubstitution',
    'LaunchConfiguration',
    'LaunchLogDir',
    'LocalSubstitution',
    'NotSubstitution',
    'NotEqualsSubstitution',
    'OrSubstitution',
    'PathJoinSubstitution',
    'PythonExpression',
    'SubstitutionFailure',
    'TextSubstitution',
    'ThisLaunchFile',
    'ThisLaunchFileDir',
]
