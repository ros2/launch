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

"""Test the default substitution interpolator."""

from launch import LaunchContext
from launch.frontend.expose import expose_substitution
from launch.frontend.parse_substitution import parse_substitution
from launch.substitutions import TextSubstitution


def test_text_only():
    subst = parse_substitution("'yes'")
    assert len(subst) == 1
    assert subst[0].perform(None) == "'yes'"
    subst = parse_substitution('"yes"')
    assert len(subst) == 1
    assert subst[0].perform(None) == '"yes"'
    subst = parse_substitution('10')
    assert len(subst) == 1
    assert subst[0].perform(None) == '10'
    subst = parse_substitution('10e4')
    assert len(subst) == 1
    assert subst[0].perform(None) == '10e4'
    subst = parse_substitution('10e4')
    assert len(subst) == 1
    assert subst[0].perform(None) == '10e4'


@expose_substitution('test')
def parse_test_substitution(data):
    if not data or len(data) > 1:
        raise RuntimeError()
    kwargs = {}
    kwargs['text'] = ''.join([i.perform(None) for i in data[0]])
    return TextSubstitution, kwargs


def test_text_with_embedded_substitutions():
    subst = parse_substitution('why_$(test asd)_asdasd_$(test bsd)')
    assert len(subst) == 4
    assert subst[0].perform(None) == 'why_'
    assert subst[1].perform(None) == 'asd'
    assert subst[2].perform(None) == '_asdasd_'
    assert subst[3].perform(None) == 'bsd'

# TODO(ivanpauno): Don't deppend on substitution parsing methods for testing the interpolator.
# Write some dummy substitutions and parsing methods instead.


def test_substitution_with_multiple_arguments():
    subst = parse_substitution('$(env what heck)')
    assert len(subst) == 1
    subst = subst[0]
    assert subst.name[0].perform(None) == 'what'
    assert subst.default_value[0].perform(None) == 'heck'


def test_escaped_characters():
    subst = parse_substitution(r'$(env what/\$\(test asd\\\)) 10 10)')
    assert len(subst) == 2
    assert subst[0].name[0].perform(None) == 'what/$(test'
    assert subst[0].default_value[0].perform(None) == r'asd\)'
    assert subst[1].perform(None) == ' 10 10)'


def test_nested_substitutions():
    subst = parse_substitution('$(env what/$(test asd) 10) 10 10)')
    assert len(subst) == 2
    assert len(subst[0].name) == 2
    assert subst[0].name[0].perform(None) == 'what/'
    assert subst[0].name[1].perform(None) == 'asd'
    assert subst[0].default_value[0].perform(None) == '10'
    assert subst[1].perform(None) == ' 10 10)'


def test_quoted_nested_substitution():
    subst = parse_substitution(
        'go_to_$(env WHERE asd)_of_$(env '
        "'something $(test 10)')"
    )
    assert len(subst) == 4
    assert subst[0].perform(None) == 'go_to_'
    assert subst[1].name[0].perform(None) == 'WHERE'
    assert subst[1].default_value[0].perform(None) == 'asd'
    assert subst[2].perform(None) == '_of_'
    assert subst[3].name[0].perform(None) == 'something '
    assert subst[3].name[1].perform(None) == '10'
    assert subst[3].default_value[0].perform(None) == ''


def test_double_quoted_nested_substitution():
    subst = parse_substitution(
        r'$(env "asd_bsd_qsd_$(test \"asd_bds\")" "$(env DEFAULT)_qsd")'
    )
    context = LaunchContext()
    assert len(subst) == 1
    assert len(subst[0].name) == 2
    assert subst[0].name[0].perform(context) == 'asd_bsd_qsd_'
    assert subst[0].name[1].perform(context) == '"asd_bds"'
    assert len(subst[0].default_value) == 2
    assert subst[0].default_value[0].name[0].perform(context) == 'DEFAULT'
    assert subst[0].default_value[0].default_value[0].perform(context) == ''
    assert subst[0].default_value[1].perform(context) == '_qsd'


def test_combining_quotes_nested_substitution():
    subst = parse_substitution(
        '$(env "asd_bsd_qsd_$(test \'asd_bds\')" \'$(env DEFAULT)_qsd\')'
    )
    context = LaunchContext()
    assert len(subst) == 1
    assert len(subst[0].name) == 2
    assert subst[0].name[0].perform(context) == 'asd_bsd_qsd_'
    assert subst[0].name[1].perform(context) == "'asd_bds'"
    assert len(subst[0].default_value) == 2
    assert subst[0].default_value[0].name[0].perform(context) == 'DEFAULT'
    assert subst[0].default_value[0].default_value[0].perform(context) == ''
    assert subst[0].default_value[1].perform(context) == '_qsd'


if __name__ == '__main__':
    test_text_only()
    test_text_with_embedded_substitutions()
    test_substitution_with_multiple_arguments()
    test_escaped_characters()
    test_nested_substitutions()
    test_quoted_nested_substitution()
    test_double_quoted_nested_substitution()
