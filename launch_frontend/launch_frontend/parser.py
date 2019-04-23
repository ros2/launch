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

"""Module for Parser methods."""

from typing import Optional
from typing import Text

import launch

from pkg_resources import iter_entry_points

from .entity import Entity
from .expose import action_parse_methods, expose_action
from .substitutions import default_substitution_interpolation

interpolation_fuctions = {
    entry_point.name: entry_point.load()
    for entry_point in iter_entry_points('launch_frontend.interpolate_substitution')
}

extensions_loaded = False


def load_parser_extensions():
    for entry_point in iter_entry_points('launch_frontend.parser_extension'):
        entry_point.load()


def parse_action(entity: Entity) -> launch.Action:
    """Parse an action, using its registered parsing method."""
    if not extensions_loaded:
        load_parser_extensions()
    if entity.type_name not in action_parse_methods:
        raise RuntimeError('Unrecognized entity of the type: {}'.format(entity.type_name))
    return action_parse_methods[entity.type_name](entity)


def parse_substitution(value: Text, frontend: Text) -> launch.SomeSubstitutionsType:
    """Parse a substitution, using its registered parsing method."""
    if not extensions_loaded:
        load_parser_extensions()
    if frontend in interpolation_fuctions:
        return interpolation_fuctions[frontend](value)
    else:
        return default_substitution_interpolation(value)


def parse_description(entity: Entity) -> launch.LaunchDescription:
    """Parse a launch description."""
    if entity.type_name != 'launch':
        raise RuntimeError('Expected \'launch\' as root tag')
    actions = [parse_action(child) for child in entity.children]
    return launch.LaunchDescription(actions)


def str_to_bool(string):
    """Convert xs::boolean to python bool."""
    if not string or string.lower() in ('0', 'false'):
        return False
    if string.lower() in ('1', 'true'):
        return True
    raise RuntimeError('Expected "true" or "false", got {}'.format(string))


def load_optional_attribute(
    kwargs: dict,
    entity: Entity,
    name: Text,
    constructor_name: Optional[Text] = None
):
    """Load an optional attribute of `entity` named `name` in `kwargs`."""
    attr = getattr(entity, name, None)
    key = name
    if constructor_name is not None:
        key = constructor_name
    if attr is not None:
        kwargs[key] = attr


@expose_action('executable')
def parse_executable(entity: Entity):
    """Parse executable tag."""
    cmd = entity.cmd
    kwargs = {}
    load_optional_attribute(kwargs, entity, 'cwd')
    load_optional_attribute(kwargs, entity, 'name')
    load_optional_attribute(kwargs, entity, 'launch-prefix', 'prefix')
    load_optional_attribute(kwargs, entity, 'output')
    shell = getattr(entity, 'shell', None)
    if shell is not None:
        kwargs['shell'] = str_to_bool(shell)
    # TODO(ivanpauno): How will predicates be handle in env?
    # Substitutions aren't allowing conditions now.
    env = getattr(entity, 'env', None)
    if env is not None:
        env = {e.name: e.value for e in env}
        kwargs['additional_env'] = env
    args = getattr(entity, 'args', None)
    args = args.split(' ') if args else []
    if not isinstance(args, list):
        args = [args]
    cmd_list = [cmd]
    cmd_list.extend(args)
    # TODO(ivanpauno): Handle predicate conditions
    return launch.actions.ExecuteProcess(
        cmd=cmd_list,
        **kwargs)


def parse_include(entity: Entity):
    """Parse a launch file to be included."""
    # TODO(ivanpauno): Should be allow to include a programmatic launch file? How?
    # TODO(ivanpauno): Create launch_ros.actions.IncludeAction, supporting namespacing.
    # TODO(ivanpauno): Handle if and unless conditions.
    loaded_entity = Entity.load(entity.file, entity.parent)
    parse_description(loaded_entity)
