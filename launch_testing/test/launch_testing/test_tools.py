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


from launch_testing.tools import basic_output_filter


def test_basic_output_filter():
    filter_fn = basic_output_filter(
        filtered_patterns=[r'.*\[listener\].*']
    )

    assert filter_fn('[listener] I heard: foo') == ''

    assert filter_fn('[talker] I said: foo') == '[talker] I said: foo'

    input_content = """\
        [listener] I heard: foo
        [listener] I heard: bar
        [listener] I heard: foobar
    """.replace('    ', '')
    output_content = ''
    assert filter_fn(input_content) == output_content

    input_content = """\
        [talker] I said: foo
        [listener] I heard: bar
        [listener] I heard: foobar
    """.replace('    ', '')
    output_content = """\
        [talker] I said: foo
    """.replace('    ', '')
    assert filter_fn(input_content) == output_content

    input_content = """\
        [talker] I said: foo
        [talker] I said: bar
        [talker] I said: foobar
    """.replace('    ', '')
    output_content = input_content
    assert filter_fn(input_content) == output_content
