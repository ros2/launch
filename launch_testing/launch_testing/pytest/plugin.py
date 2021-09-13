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
import functools
import inspect
import itertools
import warnings

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


def pytest_configure(config):
    """Inject launch_testing marker documentation."""
    config.addinivalue_line(
        'markers',
        'launch_testing: '
        'mark the test as a launch test, it will be '
        'run using the specified launch_pad.',
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
            'or a launch description, locals pair'
        )
        ld = ret
        if isinstance(ret, tuple):
            assert len(ret) == 2, wrong_ret_type_error
            ld, _ = ret
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
        raise RuntimeError(
            'launch_pytest fixtures must return a LaunchDescription '
            'containing a ReadyToTest action'
        )
    try:
        next(gen)
    except StopIteration:  # Only one ReadyToTest action must be found
        return ready_action
    raise RuntimeError(
        'launch_pytest fixtures must return a LaunchDescription '
        'containing only one ReadyToTest action'
    )


def is_valid_test_item(obj):
    """Return true if obj is a valid launch test item."""
    return (
        inspect.iscoroutinefunction(obj) or inspect.isfunction(obj)
        or inspect.isgeneratorfunction(obj) or inspect.isasyncgenfunction(obj)
    )


def need_shutdown_test_item(obj):
    """Return true if we also need to generate a shutdown test item for this object."""
    return inspect.isgeneratorfunction(obj) or inspect.isasyncgenfunction(obj)


def generate_test_items(collector, name, obj, fixturename, is_shutdown):
    """Return list of test items for the corresponding object and injects the needed fixtures."""
    items = list(collector._genfunctions(name, obj))
    for item in items:
        if fixturename in item.fixturenames:
            item.fixturenames.remove(fixturename)
        item.fixturenames.insert(0, fixturename)
        if 'launch_service' in item.fixturenames:
            item.fixturenames.remove('launch_service')
        item.fixturenames.insert(0, 'launch_service')
        if 'event_loop' in item.fixturenames:
            item.fixturenames.remove('event_loop')
        item.fixturenames.insert(0, 'event_loop')
        item._launch_testing_is_shutdown = is_shutdown
        if is_shutdown:
            # rename the items, to differentiate them from the normal test stage
            item.name = f'{name}[shutdown_test]'
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

        marker = item.get_closest_marker('launch_testing')
        if marker is not None:
            # inject the correct launch_testing fixture here
            if 'fixture' not in marker.kwargs:
                warnings.warn(
                    '"fixture" keyword argument is required in a pytest.mark.launch_testing() '
                    f'decorator: \n{get_error_context_from_obj(item.obj)}'
                )
                return None
            fixture = marker.kwargs['fixture']
            fixturename = fixture.__name__
            scope = fixture._pytestfixturefunction.scope
            is_shutdown = marker.kwargs.get('shutdown', False)
            items = generate_test_items(collector, name, obj, fixturename, is_shutdown)
            if need_shutdown_test_item(obj) and scope != 'function':
                # for function scope we only need one shutdown item
                # if not we're going to use two event loops!!!
                shutdown_items = generate_test_items(collector, name, obj, fixturename, True)
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


@pytest.mark.trylast
def pytest_collection_modifyitems(session, config, items):
    """Reorder tests, so shutdown tests happen after the corresponding fixture teardown."""

    def cmp(left, right):
        leftm = left.get_closest_marker('launch_testing')
        rightm = right.get_closest_marker('launch_testing')
        if None in (leftm, rightm):
            return 0
        fixture = leftm.kwargs['fixture']
        if fixture is not rightm.kwargs['fixture']:
            return 0
        if fixture._pytestfixturefunction.scope == 'function':
            return 0
        left_is_shutdown = int(left._launch_testing_is_shutdown)
        right_is_shutdown = int(right._launch_testing_is_shutdown)
        return left_is_shutdown - right_is_shutdown

    # python sort is guaranteed to be stable
    items.sort(key=functools.cmp_to_key(cmp))
    # This approach doesn't work, because we cannot guarantee when fixture
    # finalizers are going to be run.
    # remove the launch_testing fixture from shutdown tests
    # so launch service shutdown happens right before... by magic!!!
    for item in items:
        marker = item.get_closest_marker('launch_testing')
        if marker is not None and item._launch_testing_is_shutdown:
            item.fixturenames.remove(marker.kwargs['fixture'].__name__)


@pytest.hookimpl(hookwrapper=True, tryfirst=True)
def pytest_pyfunc_call(pyfuncitem):
    """Run launch_testing test coroutines and functions in an event loop."""
    marker = pyfuncitem.get_closest_marker('launch_testing')
    if marker is not None:
        args = {}
        func = pyfuncitem.obj
        spec = inspect.getfullargspec(func)
        shutdown_test = pyfuncitem._launch_testing_is_shutdown
        extra_args = {}
        fixture = marker.kwargs['fixture']
        scope = fixture._pytestfixturefunction.scope
        if not shutdown_test:
            fixturename = marker.kwargs['fixture'].__name__
            ld_extra_args_pair = pyfuncitem.funcargs[fixturename]
            if isinstance(ld_extra_args_pair, tuple) and len(ld_extra_args_pair) == 2:
                extra_args = ld_extra_args_pair[1]
        event_loop = pyfuncitem.funcargs['event_loop']
        ls = pyfuncitem.funcargs['launch_service']
        on_shutdown = functools.partial(
            finalize_launch_service, ls, eprefix=f'When running test {func.__name__}')
        before_test = on_shutdown if shutdown_test else None
        for name, value in extra_args.items():
            if name in itertools.chain(spec.args, spec.kwonlyargs):
                args[name] = value
        if inspect.iscoroutinefunction(func):
            pyfuncitem.obj = wrap_coroutine(func, args, event_loop, before_test)
        elif inspect.isgeneratorfunction(func):
            if scope != 'function':
                pyfuncitem.obj, pyfuncitem._launch_testing_shutdown_item.obj = (
                    wrap_generator(func, args, event_loop, on_shutdown)
                )
            else:
                pyfuncitem.obj = wrap_generator_fscope(func, args, event_loop, on_shutdown)
        elif inspect.isasyncgenfunction(func):
            if scope != 'function':
                pyfuncitem.obj, pyfuncitem._launch_testing_shutdown_item.obj = (
                    wrap_asyncgen(func, args, event_loop, on_shutdown)
                )
            else:
                pyfuncitem.obj = wrap_asyncgen_fscope(func, args, event_loop, on_shutdown)
        elif not getattr(pyfuncitem.obj, '_launch_testing_wrapped', False):
            pyfuncitem.obj = wrap_func(func, args, event_loop, before_test)
    yield


def wrap_coroutine(func, args, event_loop, before_test):
    """Return a sync wrapper around an async function to be executed in the event loop."""

    @functools.wraps(func)
    def inner(**kwargs):
        if before_test is not None:
            before_test()
        update_arguments(kwargs, args)
        coro = func(**kwargs)
        task = asyncio.ensure_future(coro, loop=event_loop)
        run_until_complete(event_loop, task)

    return inner


def wrap_func(func, args, event_loop, before_test):
    """Return a wrapper that runs the test in a separate thread while driving the event loop."""

    @functools.wraps(func)
    def inner(**kwargs):
        if before_test is not None:
            before_test()
        update_arguments(kwargs, args)
        future = event_loop.run_in_executor(None, functools.partial(func, **kwargs))
        run_until_complete(event_loop, future)
    return inner


def wrap_generator(func, args, event_loop, on_shutdown):
    """Return wrappers for the normal test and the teardown test for a generator function."""
    gen = None

    @functools.wraps(func)
    def shutdown(**kwargs):
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

    @functools.wraps(func)
    def inner(**kwargs):
        nonlocal gen
        update_arguments(kwargs, args)
        local_gen = func(**kwargs)
        future = event_loop.run_in_executor(None, lambda: next(local_gen))
        run_until_complete(event_loop, future)
        gen = local_gen

    return inner, shutdown


def wrap_generator_fscope(func, args, event_loop, on_shutdown):
    """Return wrappers for the normal test and the teardown test for a generator function."""

    @functools.wraps(func)
    def inner(**kwargs):
        update_arguments(kwargs, args)
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


def wrap_asyncgen(func, args, event_loop, on_shutdown):
    """Return wrappers for the normal test and the teardown test for an async gen function."""
    agen = None

    @functools.wraps(func)
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
    shutdown._launch_testing_wrapped = True

    @functools.wraps(func)
    def inner(**kwargs):
        nonlocal agen
        update_arguments(kwargs, args)
        local_agen = func(**kwargs)
        coro = local_agen.__anext__()
        task = asyncio.ensure_future(coro, loop=event_loop)
        run_until_complete(event_loop, task)
        agen = local_agen

    return inner, shutdown


def wrap_asyncgen_fscope(func, args, event_loop, on_shutdown):
    """Return wrappers for the normal test and the teardown test for an async gen function."""

    @functools.wraps(func)
    def inner(**kwargs):
        update_arguments(kwargs, args)
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


def update_arguments(kwargs, extra_kwargs):
    """Update kwargs with extra_kwargs, print a warning if a key overlaps."""
    overlapping_keys = kwargs.keys() & extra_kwargs.keys()
    if overlapping_keys:
        warnings.warn(
            'Argument(s) returned in launch_testing fixture has the same name than a pytest '
            'fixture. Use different names to avoid confusing errors.')

    kwargs.update(extra_kwargs)


def run_until_complete(loop, future_like):
    """Similar to `asyncio.EventLoop.run_until_complete`, but it consumes all exceptions."""
    try:
        loop.run_until_complete(future_like)
    except BaseException:
        if future_like.done() and not future_like.cancelled():
            future_like.exception()
        raise


"""Launch service fixture."""
launch_service = get_launch_service_fixture(overridable=False)
