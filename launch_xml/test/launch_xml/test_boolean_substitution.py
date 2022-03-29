# Copyright 2022 Open Source Robotics Foundation, Inc.
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

import io
import textwrap

from launch import LaunchService
from launch.frontend import Parser
from launch.utilities import perform_substitutions


def test_boolean_substitution_xml():
    xml_file = textwrap.dedent(
        r"""
        <launch>
            <let name="true_value" value="true" />
            <let name="false_value" value="false" />

            <let name="not_true" value="$(not $(var true_value))" />
            <let name="not_false" value="$(not $(var false_value))" />

            <let name="and_true_true" value="$(and $(var true_value) $(var true_value))" />
            <let name="and_true_false" value="$(and $(var true_value) $(var false_value))" />
            <let name="and_false_true" value="$(and $(var false_value) $(var true_value))" />
            <let name="and_false_false" value="$(and $(var false_value) $(var false_value))" />

            <let name="or_true_true" value="$(or $(var true_value) $(var true_value))" />
            <let name="or_true_false" value="$(or $(var true_value) $(var false_value))" />
            <let name="or_false_true" value="$(or $(var false_value) $(var true_value))" />
            <let name="or_false_false" value="$(or $(var false_value) $(var false_value))" />
        </launch>
        """
    )
    with io.StringIO(xml_file) as f:
        check_boolean_substitution(f)


def check_boolean_substitution(file):
    root_entity, parser = Parser.load(file)
    ld = parser.parse_description(root_entity)
    ls = LaunchService()
    ls.include_launch_description(ld)
    assert 0 == ls.run()

    def perform(substitution):
        return perform_substitutions(ls.context, substitution)

    sub_entries = ld.describe_sub_entities()

    not_true = sub_entries[2]
    not_false = sub_entries[3]

    and_true_true = sub_entries[4]
    and_true_false = sub_entries[5]
    and_false_true = sub_entries[6]
    and_false_false = sub_entries[7]

    or_true_true = sub_entries[8]
    or_true_false = sub_entries[9]
    or_false_true = sub_entries[10]
    or_false_false = sub_entries[11]

    assert perform(not_true.value) == 'false'
    assert perform(not_false.value) == 'true'

    assert perform(and_true_true.value) == 'true'
    assert perform(and_true_false.value) == 'false'
    assert perform(and_false_true.value) == 'false'
    assert perform(and_false_false.value) == 'false'

    assert perform(or_true_true.value) == 'true'
    assert perform(or_true_false.value) == 'true'
    assert perform(or_false_true.value) == 'true'
    assert perform(or_false_false.value) == 'false'
