# Copyright 2026 Dennis Lanov
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

"""Tests for the interaction of launch_testing with pytest's own collection filtering."""

from launch_testing.pytest import hooks
from launch_testing.pytest.hooks import LaunchTestItem

pytest_plugins = ['pytester']

TEST_BODY = 'def test_it():\n    pass\n'


def collect(pytester, *args):
    """Collect tests in pytester's directory and return the collected items by node id."""
    items, reprec = pytester.inline_genitems(*args)
    # Make sure the launch_testing plugin took part in collection.
    config = reprec.getcall('pytest_sessionstart').session.config
    assert hooks in config.pluginmanager.get_plugins()
    # A file that fails to collect yields no items, so check for errors too.
    assert reprec.getfailedcollections() == []
    return {item.nodeid: item for item in items}


def test_regular_test_discovery(pytester):
    pytester.makepyfile(test_regular=TEST_BODY)
    pytester.mkpydir('pkg')
    pytester.path.joinpath('pkg', 'test_in_pkg.py').write_text(TEST_BODY)
    pytester.makepyfile(test_launch_style='def generate_test_description():\n    pass\n')

    items = collect(pytester)

    assert 'test_regular.py::test_it' in items
    assert 'pkg/test_in_pkg.py::test_it' in items
    assert isinstance(items['test_launch_style.py::test_launch_style'], LaunchTestItem)


def test_ignore_option(pytester):
    pytester.makepyfile(test_kept=TEST_BODY, test_ignored=TEST_BODY)

    items = collect(pytester, '--ignore=test_ignored.py')

    assert list(items) == ['test_kept.py::test_it']


def test_ignore_glob_option(pytester):
    pytester.makepyfile(test_kept=TEST_BODY, test_ignored=TEST_BODY)

    items = collect(pytester, '--ignore-glob=*_ignored.py')

    assert list(items) == ['test_kept.py::test_it']


def test_conftest_collect_ignore(pytester):
    pytester.makeconftest("collect_ignore = ['test_ignored.py']")
    pytester.makepyfile(test_kept=TEST_BODY, test_ignored=TEST_BODY)

    items = collect(pytester)

    assert list(items) == ['test_kept.py::test_it']


def test_hidden_directories_are_not_recursed(pytester):
    pytester.makepyfile(test_kept=TEST_BODY)
    pytester.mkdir('.hidden')
    pytester.path.joinpath('.hidden', 'test_hidden.py').write_text(TEST_BODY)

    items = collect(pytester)

    assert list(items) == ['test_kept.py::test_it']


def test_norecursedirs(pytester):
    pytester.makeini('[pytest]\nnorecursedirs = skipped\n')
    pytester.makepyfile(test_kept=TEST_BODY)
    pytester.mkdir('skipped')
    pytester.path.joinpath('skipped', 'test_skipped.py').write_text(TEST_BODY)

    items = collect(pytester)

    assert list(items) == ['test_kept.py::test_it']


def test_launch_py_files_are_ignored(pytester):
    pytester.makepyfile(test_kept=TEST_BODY)
    # Matches the default python_files pattern, so only the plugin keeps it out.
    pytester.path.joinpath('test_example.launch.py').write_text(TEST_BODY)

    items = collect(pytester)

    assert list(items) == ['test_kept.py::test_it']


def test_logging_handlers_is_ignored(pytester):
    pytester.makeini('[pytest]\npython_files = test_*.py handlers.py\n')
    pytester.makepyfile(test_kept=TEST_BODY)
    pytester.mkdir('logging')
    pytester.path.joinpath('logging', 'handlers.py').write_text(TEST_BODY)

    items = collect(pytester)

    assert list(items) == ['test_kept.py::test_it']
