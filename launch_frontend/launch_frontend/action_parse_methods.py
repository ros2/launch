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

from .convert_text_to import str_to_bool
from .entity import Entity
from .expose import expose_action
from .parser import Parser


@expose_action('executable')
def parse_executable(entity: Entity, parser: Parser):
    """Parse executable tag."""
    cmd = parser.parse_substitution(entity.cmd)
    kwargs = {}
    cwd = getattr(entity, 'cwd', None)
    if cwd is not None:
        kwargs['cwd'] = parser.parse_substitution(cwd)
    name = getattr(entity, 'name', None)
    if name is not None:
        kwargs['name'] = parser.parse_substitution(name)
    prefix = getattr(entity, 'launch-prefix', None)
    if prefix is not None:
        kwargs['prefix'] = parser.parse_substitution(prefix)
    output = getattr(entity, 'output', None)
    if output is not None:
        kwargs['output'] = output
    shell = getattr(entity, 'shell', None)
    if shell is not None:
        kwargs['shell'] = str_to_bool(shell)
    # Conditions won't be allowed in the `env` tag.
    # If that feature is needed, `set_enviroment_variable` and
    # `unset_enviroment_variable` actions should be used.
    env = getattr(entity, 'env', None)
    if env is not None:
        env = {e.name: parser.parse_substitution(e.value) for e in env}
        kwargs['additional_env'] = env
    args = getattr(entity, 'args', None)
    # `args` is supposed to be a list separated with ' '.
    # All the found `TextSubstitution` items are split and
    # added to the list again as a `TextSubstitution`.
    # Another option: Enforce to explicetly write a list in
    # the launch file (if that's wanted)
    # In xml 'args' and 'args-sep' tags should be used.
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
    # Option 2:
    # if args is not None:
    #     if isinstance(args, Text):
    #         args = [args]
    #     args = [parser.parse_substitution(arg) for arg in args]
    # else:
    #     args = []
    cmd_list = [cmd]
    cmd_list.extend(args)
    if_cond = getattr(entity, 'if', None)
    unless_cond = getattr(entity, 'unless', None)
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
    name = parser.parse_substitution(entity.var)
    value = parser.parse_substitution(entity.value)
    return launch.actions.SetLaunchConfiguration(
        name,
        value
    )

# def parse_include(entity: Entity):
#     """Parse a launch file to be included."""
#     # TODO(ivanpauno): Should be allow to include a programmatic launch file? How?
#     # TODO(ivanpauno): Create launch_ros.actions.IncludeAction, supporting namespacing.
#     # TODO(ivanpauno): Handle if and unless conditions.
#     loaded_entity = Entity.load(entity.file, entity.parent)
#     parse_description(loaded_entity)
