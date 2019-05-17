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

"""Module for launch action parsing methods."""

import shlex

import launch
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import TextSubstitution

from .entity import Entity
from .expose import expose_action
from .parser import Parser


@expose_action('executable')
def parse_executable(entity: Entity, parser: Parser):
    """Parse executable tag."""
    cmd = parser.parse_substitution(entity.get_attr('cmd'))
    kwargs = {}
    cwd = entity.get_attr('cwd', optional=True)
    if cwd is not None:
        kwargs['cwd'] = parser.parse_substitution(cwd)
    name = entity.get_attr('name', optional=True)
    if name is not None:
        kwargs['name'] = parser.parse_substitution(name)
    prefix = entity.get_attr('launch-prefix', optional=True)
    if prefix is not None:
        kwargs['prefix'] = parser.parse_substitution(prefix)
    output = entity.get_attr('output', optional=True)
    if output is not None:
        kwargs['output'] = output
    shell = entity.get_attr('shell', types='bool', optional=True)
    if shell is not None:
        kwargs['shell'] = shell
    # Conditions won't be allowed in the `env` tag.
    # If that feature is needed, `set_enviroment_variable` and
    # `unset_enviroment_variable` actions should be used.
    env = entity.get_attr('env', types='list[Entity]', optional=True)
    if env is not None:
        # TODO(ivanpauno): Change `ExecuteProcess` api. `additional_env`
        # argument is supposed to be a dictionary with `SomeSubstitutionType`
        # keys, but `SomeSubstitutionType` is not always hashable.
        # Proposed `additional_env` type:
        #   Iterable[Tuple[SomeSubstitutionType, SomeSubstitutionsType]]
        env = {e.get_attr('name'): parser.parse_substitution(e.get_attr('value')) for e in env}
        kwargs['additional_env'] = env
    args = entity.get_attr('args', optional=True)
    # `args` is supposed to be a list separated with ' '.
    # All the found `TextSubstitution` items are split and
    # added to the list again as a `TextSubstitution`.
    # TODO(ivanpauno): Change `ExecuteProcess` api from accepting
    # `Iterable[SomeSubstitutionType]` `cmd` to `SomeSubstitutionType`.
    # After performing the substitution in `cmd`, shlex.split should be done.
    # This will also allow having a substitution which ends in more than one
    # argument.
    if args is not None:
        args = parser.parse_substitution(args)
        new_args = []
        for arg in args:
            if isinstance(arg, TextSubstitution):
                text = arg.text
                text = shlex.split(text)
                text = [TextSubstitution(text=item) for item in text]
                new_args.extend(text)
            else:
                new_args.append(arg)
        args = new_args
    else:
        args = []
    cmd_list = [cmd]
    cmd_list.extend(args)
    if_cond = entity.get_attr('if', optional=True)
    unless_cond = entity.get_attr('unless', optional=True)
    if if_cond is not None and unless_cond is not None:
        raise RuntimeError("if and unless conditions can't be usede simultaneously")
    if if_cond is not None:
        kwargs['condition'] = IfCondition(predicate_expression=if_cond)
    if unless_cond is not None:
        kwargs['condition'] = UnlessCondition(predicate_expression=unless_cond)

    return launch.actions.ExecuteProcess(
        cmd=cmd_list,
        **kwargs
    )


@expose_action('let')
def parse_let(entity: Entity, parser: Parser):
    """Parse let tag."""
    name = parser.parse_substitution(entity.get_attr('name'))
    value = parser.parse_substitution(entity.get_attr('value'))
    return launch.actions.SetLaunchConfiguration(
        name,
        value
    )
