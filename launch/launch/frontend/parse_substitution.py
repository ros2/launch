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

"""Module for parsing substitutions."""

import os
import re
from typing import Text

from ament_index_python.packages import get_package_share_directory

from lark import Lark
from lark import Token
from lark import Transformer

from .expose import instantiate_substitution
from ..substitutions import TextSubstitution


def replace_escaped_characters(data: Text) -> Text:
    """Search escaped characters and replace them."""
    return re.sub(r'\\(.)', r'\1', data)


class ExtractSubstitution(Transformer):
    """Extract a substitution."""

    def part(self, content):
        assert(len(content) == 1)
        content = content[0]
        if isinstance(content, Token):
            assert content.type.endswith('_RSTRING')
            return TextSubstitution(text=replace_escaped_characters(content.value))
        return content

    single_quoted_part = part
    double_quoted_part = part

    def value(self, parts):
        if len(parts) == 1 and isinstance(parts[0], list):
            # Deal with single and double quoted templates
            return parts[0]
        return parts

    single_quoted_value = value
    double_quoted_value = value

    def arguments(self, values):
        if len(values) > 1:
            # Deal with tail recursive argument parsing
            return [*values[0], values[1]]
        return values

    single_quoted_arguments = arguments
    double_quoted_arguments = arguments

    def substitution(self, args):
        assert len(args) >= 1
        name = args[0]
        assert isinstance(name, Token)
        assert name.type == 'IDENTIFIER'
        return instantiate_substitution(name.value, *args[1:])

    single_quoted_substitution = substitution
    double_quoted_substitution = substitution

    def fragment(self, content):
        assert len(content) == 1
        content = content[0]
        if isinstance(content, Token):
            assert content.type.endswith('_STRING')
            return TextSubstitution(text=replace_escaped_characters(content.value))
        return content

    single_quoted_fragment = fragment
    double_quoted_fragment = fragment

    def template(self, fragments):
        return fragments

    single_quoted_template = template
    double_quoted_template = template


grammar_file = os.path.join(get_package_share_directory('launch'), 'frontend', 'grammar.lark')
parser = Lark.open(grammar_file, start='template')
transformer = ExtractSubstitution()


def parse_substitution(string_value):
    if not string_value:
        # Grammar cannot deal with zero-width expressions.
        return [TextSubstitution(text=string_value)]
    tree = parser.parse(string_value)
    return transformer.transform(tree)
