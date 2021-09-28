# Copyright 2021 Open Source Robotics Foundation, Inc.
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

import asyncio
import inspect

import launch
import pytest

try:
    from _pytest.scope import Scope

    def scope_gt(scope1, scope2):
        return Scope(scope1) > Scope(scope2)
except ImportError:
    from _pytest.fixtures import scopemismatch as scope_gt


def finalize_launch_service(launch_service, eprefix=''):
    launch_service.shutdown(force_sync=True)
    loop = launch_service.event_loop
    if loop is not None and not loop.is_closed():
        rc = loop.run_until_complete(launch_service.task)
        assert rc == 0, f"{eprefix} launch service failed when finishing, return code '{rc}'"


def get_launch_service_fixture(*, scope='function', overridable=True):
    """Return a launch service fixture."""

    @pytest.fixture(scope=scope)
    def launch_service(event_loop):
        """Create an instance of the launch service for each test case."""
        ls = launch.LaunchService()
        yield ls
        finalize_launch_service(ls, eprefix='When tearing down launch_service fixture')
    if overridable:
        launch_service._launch_testing_overridable_fixture = True
        launch_service._launch_testing_fixture_scope = scope
    return launch_service


def get_event_loop_fixture(*, scope='function', overridable=True):
    """Return an event loop fixture."""

    @pytest.fixture(scope=scope)
    def event_loop():
        """Create an event loop instance for each test case."""
        loop = asyncio.get_event_loop_policy().new_event_loop()
        yield loop
        loop.close()
    if overridable:
        event_loop._launch_testing_overridable_fixture = True
        event_loop._launch_testing_fixture_scope = scope
    return event_loop


def fixture(decorated = None, *args, **kwargs):
    """
    Decorate launch_test fixtures.

    For documentation on the supported arguments, see
    https://docs.pytest.org/en/latest/reference/reference.html#pytest-fixture.
    """
    # Automagically override the event_loop and launch_testing fixtures
    # with a fixture of the correct scope.
    # This is not done if the user explicitly provided this fixture,
    # they might get an ScopeError if not correctly defined.
    scope = kwargs.get('scope', 'function')
    if scope != 'function':
        frame = inspect.stack()[1]
        mod = inspect.getmodule(frame[0])
        mod_locals = vars(mod)
        for name, getter in (
            ('launch_service', get_launch_service_fixture),
            ('event_loop', get_event_loop_fixture)
        ):
            if name in mod_locals:
                obj = mod_locals[name]
                if (
                    getattr(obj, '_launch_testing_overridable_fixture', False) and
                    scope_gt(scope, obj._launch_testing_fixture_scope)
                ):
                    mod_locals[name] = getter(scope=scope)
            else:
                mod_locals[name] = getter(scope=scope)

    def decorator(fixture_function):
        fixture_function._launch_pytest_fixture = True
        return pytest.fixture(fixture_function, *args, **kwargs)
    if decorated is None:
        return decorator
    return decorator(decorated)
