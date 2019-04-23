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

"""Module for built-in front-end substitutions."""

from typing import Text

from launch import SomeSubstitutionsType
from launch.substitutions import EnvironmentVariable
from launch.substitutions import FindExecutable


from .expose import expose_substitution, substitution_parse_methods


@expose_substitution('test')
def test(data):
    """Delete me please."""
    if len(data) > 1:
        raise AttributeError('Expected a len 1 list')
    return data[0]


@expose_substitution('list')
def parse_list(string: Text):
    """Parse a list substitution."""
    if len(string) > 1:
        raise AttributeError('Expected a len 1 list.')
    string = string[0]
    pos = string.find('sep=')
    sep = ','
    if pos == 0:
        sep, string = string[4:].split(' ', 1)
    return string.split(sep)


def parse_find_executable(executable_name: SomeSubstitutionsType):
    """Return FindExecutable substitution."""
    return FindExecutable(executable_name)


@expose_substitution('env')
def parse_env(data):
    """Return FindExecutable substitution."""
    if not data or len(data) > 2:
        raise AttributeError('env substitution expected 1 or 2 arguments')
    name = data[0]
    default = data[1] if len(data) == 2 else ''
    # print(name)
    # print(default)
    return EnvironmentVariable(name, default_value=default)


def find_no_scaped(string, substr, pos):
    """Find the first non scaped 'substr' substring in 'string', starting from 'pos'."""
    scaped_substr = '\\' + substr
    dscaped_substr = '\\' + scaped_substr
    while True:
        pos_found = string.find(substr, pos)
        pos_found_scaped = string.find(scaped_substr, pos)
        pos_found_dscaped = string.find(dscaped_substr, pos)
        is_double_scaped = pos_found_dscaped >= 0 and pos_found == pos_found_dscaped + 2
        is_scaped = pos_found_scaped >= 0 and pos_found == pos_found_scaped + 1
        is_scaped = not is_double_scaped and is_scaped
        if pos_found < 0 or not is_scaped:
            return pos_found
        pos = pos_found + 1


def replace_scapes(string):
    ret = ''
    pos = 0
    while True:
        last_pos = pos
        pos = string.find('\\', pos)
        if pos < 0:
            return ret + string[last_pos:]
        if string[pos+1] == '\\':
            ret = ret + string[last_pos:pos] + '\\'
            pos = pos + 2
            continue
        if string[pos+1:pos+3] == '$(':
            ret = ret + string[last_pos:pos] + '$('
            pos = pos + 3
            continue
        if string[pos+1] == ')':
            ret = ret + string[last_pos:pos] + ')'
            pos = pos + 2
            continue
        raise RuntimeError('Wrong backslash usage in string')


def default_substitution_interpolation(string):
    """Interpolate substitutions in a string."""
    # TODO(ivanpauno): Use 're' package to do this in a cleaner way.
    # This scans from left to right. It pushes the position
    # of the opening brackets. When it finds a closing bracket
    # it pops and substitute.
    # The output is a list of substitutions and strings.
    subst_list = []  # Substitutions list to be returned.
    pos = 0  # Position of the string when we should continue parsing.
    opening_brackets_pile = []  # Pile containing opening brackets.
    # A dict, containing the nested substitutions that have been done.
    # The key is the nesting level.
    # Each item is a list, in order to handle substitutions that takes
    # more than one argument, like $(env var default).
    nested_substitutions = dict({})
    while 1:
        ob = find_no_scaped(string, '$(', pos)
        cb = find_no_scaped(string, ')', pos)
        if ob >= 0 and ob < cb:
            # New opening bracket found
            if opening_brackets_pile:
                middle_string = None
                if pos > opening_brackets_pile[-1] + 2:
                    middle_string = string[pos:ob]
                else:
                    # Skip the key
                    _, middle_string = string[
                        opening_brackets_pile[-1]:ob].split(' ', 1)
                if middle_string:
                    try:
                        nested_substitutions[len(opening_brackets_pile)-1].append(
                            middle_string)
                    except (TypeError, KeyError):
                        nested_substitutions[len(opening_brackets_pile)-1] = \
                            [middle_string]
            opening_brackets_pile.append(ob)
            pos = ob + 2
            continue
        if cb >= 0:
            # New closing bracket found
            ob_pop = opening_brackets_pile.pop()
            subst_key, subst_value = string[ob_pop+2:cb].split(' ', 1)
            if subst_key not in substitution_parse_methods:
                # Unknown substitution
                raise RuntimeError(
                    'Invalid substitution type: {}'.format(subst_key))
            if len(opening_brackets_pile)+1 not in nested_substitutions:
                # Doesn't have a nested substitution inside.
                try:
                    nested_substitutions[len(opening_brackets_pile)].append(
                        substitution_parse_methods[subst_key]([replace_scapes(subst_value)]))
                except (TypeError, KeyError):
                    nested_substitutions[len(opening_brackets_pile)] = \
                        [substitution_parse_methods[subst_key]([replace_scapes(subst_value)])]
            else:
                # Have a nested substitution inside
                try:
                    nested_substitutions[len(opening_brackets_pile)].append(
                        substitution_parse_methods[subst_key](
                            nested_substitutions[len(opening_brackets_pile)+1])
                    )
                except (TypeError, KeyError):
                    nested_substitutions[len(opening_brackets_pile)] = [
                        substitution_parse_methods[subst_key](
                            nested_substitutions[len(opening_brackets_pile)+1])
                    ]
                del nested_substitutions[len(opening_brackets_pile)+1]
            if not opening_brackets_pile:
                # Is not nested inside other substitution
                subst_list.append(replace_scapes(string[:ob_pop]))
                subst_list.extend(nested_substitutions[0])
                string = string[cb+1:]
                pos = 0
                nested_substitutions = dict({})
            else:
                # Is still nested in other substitution
                pos = cb + 1
            continue
        if opening_brackets_pile:
            raise RuntimeError('Non matching substitution brackets.')
        return subst_list


if __name__ == '__main__':
    print(default_substitution_interpolation(
        r'hola $(test \)como\$(\\) $(test $(list 1,2,3))'
        ' $(env $(env jkl $(test msj)) bsd)'))
