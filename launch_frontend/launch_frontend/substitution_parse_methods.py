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

"""Module for launch substitution parsing methods."""

from typing import Iterable

import launch

from .expose import expose_substitution


@expose_substitution('env')
def parse_env(data: Iterable[launch.SomeSubstitutionsType]):
    """Parse EnviromentVariable substitution."""
    if not data or len(data) > 2:
        raise AttributeError('env substitution expects 1 or 2 arguments')
    name = data[0]
    kwargs = {}
    if len(data) == 2:
        kwargs['default_value'] = data[1]
    return launch.substitutions.EnvironmentVariable(name, **kwargs)


@expose_substitution('var')
def parse_var(data: Iterable[launch.SomeSubstitutionsType]):
    """Parse FindExecutable substitution."""
    if not data or len(data) > 2:
        raise AttributeError('var substitution expects 1 or 2 arguments')
    name = data[0]
    kwargs = {}
    if len(data) == 2:
        kwargs['default'] = data[1]
    return launch.substitutions.LaunchConfiguration(name, **kwargs)


@expose_substitution('find-exec')
def parse_find_exec(data: Iterable[launch.SomeSubstitutionsType]):
    """Parse FindExecutable substitution."""
    if not data or len(data) > 1:
        raise AttributeError('find-exec substitution expects 1 argument')
    name = data[0]
    return launch.substitutions.FindExecutable(name=name)
