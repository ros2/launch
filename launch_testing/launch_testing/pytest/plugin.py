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
from collections.abc import Sequence
import functools
import inspect

from _pytest.outcomes import fail
from _pytest.outcomes import skip

import launch
import launch_testing

import pytest

from .fixture import finalize_launch_service
from .fixture import get_launch_service_fixture

"""
launch_testing native pytest based implementation.
"""


try:
    from _pytest.python import transfer_markers
except ImportError:  # Pytest 4.1.0 removes the transfer_marker api (#104)

    def transfer_markers(*args, **kwargs):  # noqa
        """Noop when over pytest 4.1.0"""
        pass


class LaunchTestWarning(pytest.PytestWarning):
    """Raised in this plugin to warn users."""

    pass


def pytest_configure(config):
    """Inject launch_testing marker documentation."""
    config.addinivalue_line(
        'markers',
        'launch_testing: '
        'mark the test as a launch test, it will be '
        'run using the specified launch_pad.',
    )


# TODO(ivanpauno): Deduplicate with launch_testing
def iterate_ready_to_test_actions(entities):
    """Search recursively LaunchDescription entities for all ReadyToTest actions."""
    for entity in entities:
        if isinstance(entity, launch_testing.actions.ReadyToTest):
            yield entity
        yield from iterate_ready_to_test_actions(
            entity.describe_sub_entities()
        )
        for conditional_sub_entity in entity.describe_conditional_sub_entities():
            yield from iterate_ready_to_test_actions(
                conditional_sub_entity[1]
            )


def get_ready_to_test_action(launch_description):
    """Extract the ready to test action from the launch description."""
    gen = (e for e in iterate_ready_to_test_actions(launch_description.entities))
    try:
        ready_action = next(gen)
    except StopIteration:  # No ReadyToTest action found
        ready_action = launch_testing.actions.ReadyToTest()
        launch_description.append(ready_action)
        return ready_action
    try:
        next(gen)
    except StopIteration:  # Only one ReadyToTest action must be found
        return ready_action
    raise RuntimeError(
        'launch_pytest fixtures must return a LaunchDescription '
        'containing only one ReadyToTest action'
    )


@pytest.hookimpl(hookwrapper=True, tryfirst=True)
def pytest_fixture_setup(fixturedef, request):
    """Set up launch service for all launch_pytest fixtures."""
    if getattr(fixturedef.func, '_launch_pytest_fixture', False):
        eprefix = f"When running launch_pytest fixture '{fixturedef.func.__name__}':"
        ls = request.getfixturevalue('launch_service')
        event_loop = request.getfixturevalue('event_loop')
        # get the result of the launch fixture, we take advantage of other wrappers this way
        outcome = yield
        ret = outcome.get_result()
        wrong_ret_type_error = (
            f'{eprefix} return value must be either a launch description '
            'or a sequence which first item is a launch description'
        )
        ld = ret
        if isinstance(ret, Sequence):
            assert len(ret) > 0, wrong_ret_type_error
            ld = ret[0]
        assert isinstance(ld, launch.LaunchDescription), wrong_ret_type_error
        ls.include_launch_description(ld)
        run_async_task = event_loop.create_task(ls.run_async(
            # TODO(ivanpauno): maybe this could be configurable (?)
            shutdown_when_idle=True
        ))
        ready = get_ready_to_test_action(ld)
        asyncio.set_event_loop(event_loop)
        event = asyncio.Event()
        ready._add_callback(lambda: event.set())

        fixturedef.addfinalizer(functools.partial(finalize_launch_service, ls, eprefix=eprefix))
        run_until_complete(event_loop, event.wait())
        # this is guaranteed by the current run_async() implementation, let's check it just in case
        # it changes in the future
        assert ls.event_loop is event_loop
        assert ls.context.asyncio_loop is event_loop
        assert ls.task is run_async_task
        return
    yield


def is_launch_test(item):
    """Return `True` if the item is a launch test."""
    mark = item.get_closest_marker('launch_testing')
    return mark is not None


def is_launch_test_mark_valid(item):
    """
    Return `True` if the item is a launch test.

    If not, a warning and a skip mark will be added to the item.
    """
    kwargs = item.get_closest_marker('launch_testing').kwargs
    ret = 'fixture' in kwargs and kwargs['fixture'] is not None
    if not ret:
        msg = (
                    '"fixture" keyword argument is required in a pytest.mark.launch_testing() '
                    f'decorator')
        item.warn(LaunchTestWarning(msg))
        item.add_marker(pytest.mark.skip(msg))
    return ret


def has_shutdown_kwarg(item):
    """Return `True` if the launch test shutdown kwarg is true."""
    return item.get_closest_marker('launch_testing').kwargs.get('shutdown', False)


def get_launch_test_fixture(item):
    """Return the launch test fixture name, `None` if this isn't a launch test."""
    mark = item.get_closest_marker('launch_testing')
    if mark is None:
        return None
    try:
        return mark.kwargs['fixture']
    except KeyError:
        return None


def get_launch_test_fixturename(item):
    """Return the launch test fixture name, `None` if this isn't a launch test."""
    fixture = get_launch_test_fixture(item)
    return None if fixture is None else fixture.__name__


def is_valid_test_item(obj):
    """Return true if obj is a valid launch test item."""
    return (
        inspect.iscoroutinefunction(obj) or inspect.isfunction(obj)
        or inspect.isgeneratorfunction(obj) or inspect.isasyncgenfunction(obj)
    )


def need_shutdown_test_item(obj):
    """Return true if we also need to generate a shutdown test item for this object."""
    return inspect.isgeneratorfunction(obj) or inspect.isasyncgenfunction(obj)


def generate_test_items(collector, name, obj, fixturename, is_shutdown, needs_renaming):
    """Return list of test items for the corresponding object and injects the needed fixtures."""
    # Inject all needed fixtures.
    # We use the `usefixtures` pytest mark instead of injecting them in fixturenames
    # directly, because the second option doesn't handle parameterized launch fixtures
    # correctly.
    # We also need to inject the mark before calling _genfunctions(),
    # so it actually returns many items for parameterized launch fixtures.
    fixtures_to_inject = (fixturename, 'launch_service', 'event_loop')
    pytest.mark.usefixtures(*fixtures_to_inject)(obj)
    items = list(collector._genfunctions(name, obj))
    for item in items:
        # Mark shutdown tests correctly
        item._launch_testing_is_shutdown = is_shutdown
        if is_shutdown and needs_renaming:
            # rename the items, to differentiate them from the normal test stage
            item.name = f'{item.name}[shutdown_test]'
    return items


# Adapted from https://github.com/pytest-dev/pytest-asyncio,
# see their license https://github.com/pytest-dev/pytest-asyncio/blob/master/LICENSE.
@pytest.mark.tryfirst
def pytest_pycollect_makeitem(collector, name, obj):
    """Collect coroutine based launch tests."""
    if collector.funcnamefilter(name) and is_valid_test_item(obj):
        item = pytest.Function.from_parent(collector, name=name)

        # Due to how pytest test collection works, module-level pytestmarks
        # are applied after the collection step. Since this is the collection
        # step, we look ourselves.
        transfer_markers(obj, item.cls, item.module)
        item = pytest.Function.from_parent(collector, name=name)  # To reload keywords.

        if is_launch_test(item):
            if not is_launch_test_mark_valid(item):
                # return an item with a warning that's going to be skipped
                return [item]
            fixture = get_launch_test_fixture(item)
            fixturename = fixture.__name__
            scope = fixture._pytestfixturefunction.scope
            is_shutdown = has_shutdown_kwarg(item)
            items = generate_test_items(collector, name, obj, fixturename, is_shutdown, False)
            if need_shutdown_test_item(obj) and scope != 'function':
                # for function scope we only need one shutdown item
                # if not we're going to use two event loops!!!
                shutdown_items = generate_test_items(collector, name, obj, fixturename, True, True)
                for item, shutdown_item in zip(items, shutdown_items):
                    item._launch_testing_shutdown_item = shutdown_item
                items.extend(shutdown_items)
            return items


def get_error_context_from_obj(obj):
    """Return formatted information of the object location."""
    try:
        fspath = inspect.getsourcefile(obj)
    except TypeError:
        return 'location information of the object not found'
    try:
        lines, lineno = inspect.getsourcelines(obj)
    except IOError:
        return f'file {fspath}: source code not available'
    error_msg = f'file {fspath}, line {lineno}'
    for line in lines:
        line = line.rstrip()
        error_msg += f'\n  {line}'
        if line.lstrip().startswith('def'):
            break
    return error_msg


def is_shutdown_test(item):
    """Return `True` if the item is a launch test."""
    return getattr(item, '_launch_testing_is_shutdown', False)


def is_same_launch_test_fixture(left_item, right_item):
    """Return `True` if both items are using the same fixture with the same parameters."""
    lfn = get_launch_test_fixture(left_item)
    rfn = get_launch_test_fixture(right_item)
    if None in (lfn, rfn):
        return False
    if lfn != rfn:
        return False
    if lfn._pytestfixturefunction.scope == 'function':
        return False
    name = lfn.__name__

    def get_fixture_params(item):
        if getattr(item, 'callspec', None) is None:
            return None
        return item.callspec.params.get(name, None)
    return get_fixture_params(left_item) == get_fixture_params(right_item)


@pytest.mark.trylast
def pytest_collection_modifyitems(session, config, items):
    """Move shutdown tests after normal tests."""

    def enumerate_reversed(sequence):
        # reversed(enumerate(sequence)), doesn't work
        # here a little generator for that
        n = len(sequence) - 1
        for elem in sequence[::-1]:
            yield n, elem
            n -= 1

    for i, item in enumerate_reversed(items):
        # This algo has worst case Nitems * Nlaunchtestitems time complexity.
        # We could probably find something better, but it's not that easy because:
        # - Tests using the same launch fixture might not be already grouped,
        #   i.e. there might be a test not using the fixture in the middle.
        #   TODO(ivanpauno): Check if that's not actually guaranteed.
        #   If that's the case, we can easily modify this to have Nlaunchtestitems complexity.
        # - There's no strict partial order relation we can use.
        #
        # iterate in reverse order, so the order of shutdown item is stable
        if not is_launch_test(item):
            continue
        new_position = None
        for j, other in enumerate(items[i+1:]):
            if (
                is_same_launch_test_fixture(item, other)
                and is_shutdown_test(item) and not is_shutdown_test(other)
            ):
                new_position = i + 1 + j
                print(f'moving {item.name} after {other.name}: {i} {i+j+1}')
        if new_position is not None:
            items.insert(new_position, items.pop(i))
            print([item.name for item in items])


@pytest.hookimpl(hookwrapper=True, tryfirst=True)
def pytest_pyfunc_call(pyfuncitem):
    """Run launch_testing test coroutines and functions in an event loop."""
    if is_launch_test(pyfuncitem):
        func = pyfuncitem.obj
        shutdown_test = is_shutdown_test(pyfuncitem)
        fixture = get_launch_test_fixture(pyfuncitem)
        scope = fixture._pytestfixturefunction.scope
        event_loop = pyfuncitem.funcargs['event_loop']
        ls = pyfuncitem.funcargs['launch_service']
        on_shutdown = functools.partial(
            finalize_launch_service, ls, eprefix=f'When running test {func.__name__}')
        before_test = on_shutdown if shutdown_test else None
        if inspect.iscoroutinefunction(func):
            pyfuncitem.obj = wrap_coroutine(func, event_loop, before_test)
        elif inspect.isgeneratorfunction(func):
            if scope != 'function':
                shutdown_item = pyfuncitem._launch_testing_shutdown_item
                pyfuncitem.obj, shutdown_item.obj = (
                    wrap_generator(func, event_loop, on_shutdown)
                )
                shutdown_item._fixtureinfo = shutdown_item.session._fixturemanager.getfixtureinfo(
                    shutdown_item, shutdown_item.obj, shutdown_item.cls, funcargs=True)
            else:
                pyfuncitem.obj = wrap_generator_fscope(func, event_loop, on_shutdown)
        elif inspect.isasyncgenfunction(func):
            if scope != 'function':
                shutdown_item = pyfuncitem._launch_testing_shutdown_item
                pyfuncitem.obj, shutdown_item.obj = (
                    wrap_asyncgen(func, event_loop, on_shutdown)
                )
                shutdown_item._fixtureinfo = shutdown_item.session._fixturemanager.getfixtureinfo(
                    shutdown_item, shutdown_item.obj, shutdown_item.cls, funcargs=True)
            else:
                pyfuncitem.obj = wrap_asyncgen_fscope(func, event_loop, on_shutdown)
        elif not getattr(pyfuncitem.obj, '_launch_testing_wrapped', False):
            pyfuncitem.obj = wrap_func(func, event_loop, before_test)
    yield


def run_until_complete(loop, future_like):
    """Similar to `asyncio.EventLoop.run_until_complete`, but it consumes all exceptions."""
    try:
        loop.run_until_complete(future_like)
    except BaseException:
        if future_like.done() and not future_like.cancelled():
            future_like.exception()
        raise


def wrap_coroutine(func, event_loop, before_test):
    """Return a sync wrapper around an async function to be executed in the event loop."""

    @functools.wraps(func)
    def inner(**kwargs):
        if before_test is not None:
            before_test()
        coro = func(**kwargs)
        task = asyncio.ensure_future(coro, loop=event_loop)
        run_until_complete(event_loop, task)

    return inner


def wrap_func(func, event_loop, before_test):
    """Return a wrapper that runs the test in a separate thread while driving the event loop."""

    @functools.wraps(func)
    def inner(**kwargs):
        if before_test is not None:
            before_test()
        future = event_loop.run_in_executor(None, functools.partial(func, **kwargs))
        run_until_complete(event_loop, future)
    return inner


def wrap_generator(func, event_loop, on_shutdown):
    """Return wrappers for the normal test and the teardown test for a generator function."""
    gen = None

    def shutdown():
        nonlocal gen
        if gen is None:
            skip('shutdown test skipped because the test failed before')
        on_shutdown()
        try:
            next(gen)
        except StopIteration:
            return
        fail(
            'launch tests using a generator function must stop iteration after yielding once',
            pytrace=False
        )
    shutdown._launch_testing_wrapped = True
    shutdown.__name__ = f'{func.__name__}[shutdown]'

    @functools.wraps(func)
    def inner(**kwargs):
        nonlocal gen
        local_gen = func(**kwargs)
        future = event_loop.run_in_executor(None, lambda: next(local_gen))
        run_until_complete(event_loop, future)
        gen = local_gen

    return inner, shutdown


def wrap_generator_fscope(func, event_loop, on_shutdown):
    """Return wrappers for the normal test and the teardown test for a generator function."""

    @functools.wraps(func)
    def inner(**kwargs):
        gen = func(**kwargs)
        future = event_loop.run_in_executor(None, lambda: next(gen))
        run_until_complete(event_loop, future)
        on_shutdown()
        try:
            next(gen)
        except StopIteration:
            return
        fail(
            'launch tests using a generator function must stop iteration after yielding once',
            pytrace=False
        )

    return inner


def wrap_asyncgen(func, event_loop, on_shutdown):
    """Return wrappers for the normal test and the teardown test for an async gen function."""
    agen = None

    def shutdown(**kwargs):
        nonlocal agen
        if agen is None:
            skip('shutdown test skipped because the test failed before')
        on_shutdown()
        try:
            coro = agen.__anext__()
            task = asyncio.ensure_future(coro, loop=event_loop)
            run_until_complete(event_loop, task)
        except StopAsyncIteration:
            return
        fail(
            'launch tests using an async gen function must stop iteration after yielding once',
            pytrace=False
        )
    shutdown.__name__ = f'{func.__name__}[shutdown]'
    shutdown._launch_testing_wrapped = True

    @functools.wraps(func)
    def inner(**kwargs):
        nonlocal agen
        local_agen = func(**kwargs)
        coro = local_agen.__anext__()
        task = asyncio.ensure_future(coro, loop=event_loop)
        run_until_complete(event_loop, task)
        agen = local_agen

    return inner, shutdown


def wrap_asyncgen_fscope(func, event_loop, on_shutdown):
    """Return wrappers for the normal test and the teardown test for an async gen function."""

    @functools.wraps(func)
    def inner(**kwargs):
        agen = func(**kwargs)
        coro = agen.__anext__()
        task = asyncio.ensure_future(coro, loop=event_loop)
        run_until_complete(event_loop, task)
        on_shutdown()
        try:
            coro = agen.__anext__()
            task = asyncio.ensure_future(coro, loop=event_loop)
            run_until_complete(event_loop, task)
        except StopAsyncIteration:
            return
        fail(
            'launch tests using an async gen function must stop iteration after yielding once',
            pytrace=False
        )

    return inner


"""Launch service fixture."""
launch_service = get_launch_service_fixture(overridable=False)
