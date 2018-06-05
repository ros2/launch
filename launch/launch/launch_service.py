# Copyright 2018 Open Source Robotics Foundation, Inc.
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

"""Module for the LaunchService class."""

import asyncio
import collections
import logging
import threading
from typing import Optional
from typing import Set

import osrf_pycommon.process_utils

from .event import Event
from .event_handlers import OnIncludeLaunchDescription
from .event_handlers import OnShutdown
from .events import IncludeLaunchDescription
from .events import Shutdown
from .launch_context import LaunchContext
from .launch_description import LaunchDescription
from .launch_description_entity import LaunchDescriptionEntity
from .some_actions_type import SomeActionsType
from .utilities import visit_all_entities_and_collect_futures

_logger = logging.getLogger(name='launch.LaunchService')


class LaunchService:
    """Service that manages the event loop and runtime for launched system."""

    def __init__(self, *, shutdown_when_idle=True, debug=False):
        """
        Constructor.

        :param: shutdown_when_idle if True (default), the service will shutdown when idle
        :param: debug if True (not default), asyncio the logger are seutp for debug
        """
        self.__shutdown_when_idle = shutdown_when_idle

        # Setup logging and debugging.
        logging.basicConfig(
            level=logging.INFO,
            format='[%(msecs)d] [%(levelname)s] [%(name)s]: %(msg)s',
        )
        self.__debug = debug
        if self.__debug:
            logging.getLogger().setLevel(logging.DEBUG)

        # Setup context and register a built-in event handler for bootstrapping.
        self.__context = LaunchContext()
        self.__context.register_event_handler(OnIncludeLaunchDescription())
        self.__context.register_event_handler(OnShutdown(on_shutdown=self.__on_shutdown))

        # Setup storage for state.
        self._entity_future_pairs = []

        # Used to prevent run() being called from multiple threads.
        self.__running_lock = threading.Lock()
        self.__running = False

        # Used to allow asynchronous use of self.__loop_from_run_thread without
        # it being set to None by run() as it exits.
        self.__loop_from_run_thread_lock = threading.RLock()
        self.__loop_from_run_thread = None

        # Used to indicate when shutdown() has been called.
        self.__shutting_down = False

    def emit_event(self, event: Event) -> None:
        """
        Emit an event synchronously and thread-safely.

        If the LaunchService is not running, the event is queued until it is.
        """
        with self.__loop_from_run_thread_lock:
            if self.__loop_from_run_thread is not None:
                # loop is in use, asynchronously emit the event
                asyncio.run_coroutine_threadsafe(
                    self.__context.emit_event(event),
                    self.__loop_from_run_thread
                )
            else:
                # loop is not in use, synchronously emit the event, and it will be processed later
                self.__context.emit_event_sync(event)

    def include_launch_description(self, launch_description: LaunchDescription) -> None:
        """
        Evaluate a given LaunchDescription and visits all of its entities.

        This method is thread-safe.
        """
        self.emit_event(IncludeLaunchDescription(launch_description))

    def _prune_and_count_entity_future_pairs(self):
        needs_prune = False
        for pair in self._entity_future_pairs:
            if pair[1].done():
                needs_prune = True
        if needs_prune:
            self._entity_future_pairs = \
                [pair for pair in self._entity_future_pairs if not pair[1].done()]
        return len(self._entity_future_pairs)

    def _is_idle(self):
        number_of_entity_future_pairs = self._prune_and_count_entity_future_pairs()
        return number_of_entity_future_pairs == 0 and self.__context._event_queue.empty()

    async def _process_one_event(self) -> None:
        next_event = await self.__context._event_queue.get()
        await self.__process_event(next_event)

    async def __process_event(self, event: Event) -> None:
        _logger.debug("processing event: '{}'".format(event))
        # TODO(wjwwood): do something with async event handlers, either drop them or use them
        for event_handler in tuple(self.__context._event_handlers):
            if event_handler.matches(event):
                _logger.debug(
                    "processing event: '{}' ✓ '{}'".format(event, event_handler))
                self.__context._push_locals()
                entities = event_handler.handle(event, self.__context)
                entities = entities if isinstance(entities, collections.Iterable) else (entities,)
                for entity in [e for e in entities if e is not None]:
                    from .utilities import is_a_subclass
                    if not is_a_subclass(entity, LaunchDescriptionEntity):
                        raise RuntimeError(
                            "expected a LaunchDescriptionEntity from event_handler, got '{}'"
                            .format(entity)
                        )
                    self._entity_future_pairs.extend(
                        visit_all_entities_and_collect_futures(entity, self.__context))
                self.__context._pop_locals()
            else:
                pass
                # _logger.debug(
                #     "processing event: '{}' x '{}'".format(event, event_handler))

    async def __run_loop(self) -> None:
        while True:
            # Check if we're idle, i.e. no on-going entities (actions) or events in the queue
            is_idle = self._is_idle()  # self._entity_future_pairs is pruned here
            if not self.__shutting_down and self.__shutdown_when_idle and is_idle:
                self._shutdown(reason='idle', due_to_sigint=False)

            process_one_event_task = self.__loop_from_run_thread.create_task(
                self._process_one_event())
            if self.__shutting_down:
                # If shutting down and idle then we're done.
                if is_idle:
                    process_one_event_task.cancel()
                    return
                else:
                    entity_futures = [pair[1] for pair in self._entity_future_pairs]
                    entity_futures.append(process_one_event_task)
                    done: Set[asyncio.Future] = set()
                    while not done:
                        done, pending = await asyncio.wait(
                            entity_futures,
                            loop=self.__loop_from_run_thread,
                            timeout=1.0,
                            return_when=asyncio.FIRST_COMPLETED)
                        if not done:
                            _logger.debug('still waiting on futures: {}'.format(entity_futures))
            else:
                await process_one_event_task

    def run(self) -> None:
        """
        Start the event loop and visit all entities of all included LaunchDescriptions.

        This should only ever be run from a single thread.
        """
        # Make sure this has not been called in multiple threads.
        with self.__running_lock:
            if self.__running:
                raise RuntimeError('LaunchService.run() called from multiple threads concurrently.')
            self.__running = True

        # Acquire the lock and initialize the asyncio loop.
        with self.__loop_from_run_thread_lock:
            self.__loop_from_run_thread = osrf_pycommon.process_utils.get_loop()
            if self.__debug:
                self.__loop_from_run_thread.set_debug(True)
            self.__context._set_asyncio_loop(self.__loop_from_run_thread)

        # Run the asyncio loop over the main coroutine that processes events.
        try:
            sigint_received = False
            run_loop_task = self.__loop_from_run_thread.create_task(self.__run_loop())
            while not run_loop_task.done():
                try:
                    self.__loop_from_run_thread.run_until_complete(run_loop_task)
                except KeyboardInterrupt:
                    base_msg = 'user interrupted with ctrl-c (SIGINT)'
                    if not sigint_received:
                        _logger.warn(base_msg)
                        self._shutdown(reason='ctrl-c (SIGINT)', due_to_sigint=True)
                        sigint_received = True
                    else:
                        _logger.warn('{} again, terminating...'.format(base_msg))
                        run_loop_task.cancel()
        finally:
            # No matter what happens, unset the loop and set running to false.
            with self.__loop_from_run_thread_lock:
                self.__shutting_down = False
                self.__loop_from_run_thread = None
                self.__context._set_asyncio_loop(None)
            with self.__running_lock:
                self.__running = False

    def __on_shutdown(self, event: Event, context: LaunchContext) -> Optional[SomeActionsType]:
        self.__shutting_down = True
        return None

    def _shutdown(self, *, reason, due_to_sigint):
        if not self.__shutting_down:
            self.emit_event(Shutdown(reason=reason, due_to_sigint=due_to_sigint))
        self.__shutting_down = True

    def shutdown(self) -> None:
        """
        Shutdown all on-going activities and then stop the asyncio run loop.

        This will cause LaunchService.run() to eventually exit.

        Does nothing if LaunchService.run() is not running in another thread.

        This method is thread-safe.
        """
        with self.__loop_from_run_thread_lock:
            if self.__loop_from_run_thread is not None:
                self._shutdown(reason='LaunchService.shutdown() called', due_to_sigint=False)
