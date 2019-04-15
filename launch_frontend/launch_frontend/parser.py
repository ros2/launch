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

import launch

from .entity import Entity


def str_to_bool(string):
    """Convert xs::boolean to python bool."""
    if not string or string.lower() in ('0', 'false'):
        return False
    if string.lower() in ('1', 'true'):
        return True
    raise RuntimeError('Expected "true" or "false", got {}'.format(string))


def get_dictionary_from_key_value_pairs(pairs):
    """Get dictionary from key-value pairs."""
    if not pairs:
        return None
    return {pair.name: pair.value for pair in pairs}


def parse_executable(entity: Entity):
    """Parse executable tag."""
    cmd = entity.cmd
    cwd = getattr(entity, 'cwd', None)
    name = getattr(entity, 'name', None)
    shell = str_to_bool(getattr(entity, 'shell', None))
    prefix = getattr(entity, 'launch-prefix', None)
    output = getattr(entity, 'output', None)
    args = getattr(entity, 'args', None)
    args = args.split(' ') if args else []
    if not type(args) == list:
        args = [args]
    # TODO(ivanpauno): How will predicates be handle in env?
    # Substitutions aren't allowing conditions now.
    env = get_dictionary_from_key_value_pairs(getattr(entity, 'env', None))

    cmd_list = [cmd]
    cmd_list.extend(args)
    # TODO(ivanpauno): Handle predicate conditions
    return launch.actions.ExecuteProcess(
        cmd=cmd_list,
        cwd=cwd,
        env=env,
        name=name,
        shell=shell,
        prefix=prefix,
        output=output)
