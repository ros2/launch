# Copyright 2015 Open Source Robotics Foundation, Inc.
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
from collections import OrderedDict
import os
import signal
import sys
import threading

from launch.exit_handler import ExitHandlerContext
from launch.launch import LaunchState
from launch.protocol import SubprocessProtocol
from launch.task import TaskState


class DefaultLauncher(object):

    def __init__(self, name_prefix='', sigint_timeout=3):
        self.name_prefix = name_prefix
        self.sigint_timeout = sigint_timeout
        self.task_descriptors = []
        self.print_mutex = threading.Lock()

    def add_launch_descriptor(self, launch_descriptor):
        for task_descriptor in launch_descriptor.task_descriptors:
            # automatic naming if not specified
            if task_descriptor.name is None:
                name = str(len(self.task_descriptors))
                if name in [p.name for p in self.task_descriptors]:
                    raise RuntimeError("Process name '%s' already used" % name)
                task_descriptor.name = name

            self.task_descriptors.append(task_descriptor)

    def launch(self):
        loop = asyncio.get_event_loop()
        returncode = loop.run_until_complete(self._run())
        loop.close()

        return returncode

    @asyncio.coroutine
    def _run(self):
        launch_state = LaunchState()
        for p in self.task_descriptors:
            p.task_state = TaskState()

        # start all processes and collect their exit futures
        all_futures = OrderedDict()
        for index, p in enumerate(self.task_descriptors):
            if 'output_handler' in dir(p):
                p.output_handler.set_print_mutex(self.print_mutex)
                p.output_handler.set_line_prefix('[%s] ' % p.name)

            if 'protocol' in dir(p):
                yield from self._spawn_process(index)
                all_futures[p.protocol.exit_future] = index
            else:
                future = asyncio.async(p.coroutine)
                all_futures[future] = index

        while True:
            # skip if no more processes to run
            if not all_futures:
                break

            # wait for any process to finish
            kwargs = {
                'return_when': asyncio.FIRST_COMPLETED,
            }
            # when the event loop run does not run in the main thread
            # wake up frequently and check if any subprocess has exited
            if not isinstance(threading.current_thread(), threading._MainThread):
                kwargs['timeout'] = 0.5
            yield from asyncio.wait(all_futures.keys(), **kwargs)

            # when the event loop run does not run in the main thread
            # use custom logic to detect that subprocesses have exited
            if not isinstance(threading.current_thread(), threading._MainThread):
                for index, p in enumerate(self.task_descriptors):
                    # only consider not yet done tasks
                    if index not in all_futures.values():
                        continue
                    # only subprocesses need special handling
                    if 'transport' not in dir(p):
                        continue
                    # transport.get_pid() sometimes failed due to transport._proc being None
                    proc = p.transport.get_extra_info('subprocess')
                    pid = proc.pid
                    try:
                        pid, proc_rc = os.waitpid(pid, os.WNOHANG)
                    except ChildProcessError:
                        continue
                    if pid == 0:
                        # subprocess is still running
                        continue
                    p.returncode = proc_rc
                    # trigger syncio internal process exit callback
                    p.transport._process_exited(proc_rc)

            # collect done futures
            done_futures = [future for future in all_futures.keys() if future.done()]

            # collect return code
            restart_indices = []
            for future in done_futures:
                index = all_futures[future]
                p = self.task_descriptors[index]

                # collect return code / exception from coroutine
                if 'coroutine' in dir(p):
                    exp = future.exception()
                    if exp:
                        p.task_state.exception = exp
                        p.task_state.returncode = 1
                        # print traceback with "standard" format
                        with self.print_mutex:
                            print('(%s)' % p.name, 'Traceback (most recent call last):',
                                  file=sys.stderr)
                            for frame in future.get_stack():
                                filename = frame.f_code.co_filename
                                print('(%s)' % p.name, '  File "%s", line %d, in %s' %
                                      (filename, frame.f_lineno, frame.f_code.co_name),
                                      file=sys.stderr)
                                import linecache
                                linecache.checkcache(filename)
                                line = linecache.getline(filename, frame.f_lineno, frame.f_globals)
                                print('(%s)' % p.name, '    ' + line.strip(), file=sys.stderr)
                            print('(%s) %s: %s' % (p.name, type(exp).__name__, str(exp)),
                                  file=sys.stderr)
                    else:
                        result = future.result()
                        p.task_state.returncode = result
                    self._process_message(p, 'rc ' + str(p.task_state.returncode))

                # close transport
                if 'protocol' in dir(p):
                    self._close_process(p)

                # remove future
                del all_futures[future]

                # call exit handler of done descriptors
                context = ExitHandlerContext(launch_state, p.task_state)
                p.exit_handler(context)
                if p.task_state.restart:
                    restart_indices.append(index)

            if launch_state.teardown:
                with self.print_mutex:
                    print('() tear down')
                break

            # restart processes if requested
            for index in restart_indices:
                p = self.task_descriptors[index]
                if 'protocol' in dir(p):
                    p.task_states[index].restart_count += 1
                    yield from self._spawn_process(index)
                    all_futures[p.protocol.exit_future] = index

        # terminate all remaining processes
        if all_futures:

            # sending SIGINT to remaining processes
            for index in all_futures.values():
                p = self.task_descriptors[index]
                if 'transport' in dir(p):
                    self._process_message(p, 'signal SIGINT')
                    try:
                        p.transport.send_signal(signal.SIGINT)
                    except ProcessLookupError:
                        pass

            yield from asyncio.wait(all_futures.keys(), timeout=self.sigint_timeout)

            # sending SIGINT to remaining processes
            for index in all_futures.values():
                p = self.task_descriptors[index]
                if 'protocol' in dir(p):
                    if not p.protocol.exit_future.done():
                        self._process_message(p, 'signal SIGTERM')
                        try:
                            p.transport.send_signal(signal.SIGTERM)
                        except ProcessLookupError:
                            pass

            yield from asyncio.wait(all_futures.keys())

            # close all remaining processes
            for index in all_futures.values():
                p = self.task_descriptors[index]
                if 'transport' in dir(p):
                    self._close_process(p)

            # call exit handler of remaining descriptors
            for index in all_futures.values():
                p = self.task_descriptors[index]
                context = ExitHandlerContext(launch_state, p.task_state)
                p.exit_handler(context)

        if launch_state.returncode is None:
            launch_state.returncode = 0
        return launch_state.returncode

    def _spawn_process(self, index):
        p = self.task_descriptors[index]
        p.output_handler.process_init()
        kwargs = {}
        if p.output_handler.support_stderr2stdout():
            kwargs['stderr'] = asyncio.subprocess.STDOUT
        loop = asyncio.get_event_loop()
        transport, protocol = yield from loop.subprocess_exec(
            lambda: SubprocessProtocol(p.output_handler),
            *p.cmd,
            **kwargs)
        p.transport = transport
        p.protocol = protocol

        output_handler_description = p.output_handler.get_description()
        if 'stderr' in kwargs:
            output_handler_description = 'stderr > stdout, ' + output_handler_description

        self._process_message(
            p, 'pid %d: %s (%s)' % (transport.get_pid(), p.cmd, output_handler_description))

    def _close_process(self, process_descriptor):
        p = process_descriptor
        p.transport.close()
        p.task_state.returncode = p.transport.get_returncode()
        self._process_message(p, 'rc %d' % p.task_state.returncode)
        p.output_handler.process_cleanup()

    def _process_message(self, process_descriptor, message):
        p = process_descriptor

        with self.print_mutex:
            print('(%s)' % p.name, message)
        lines = (message + '\n').encode()
        if 'output_handler' in dir(p):
            p.output_handler.on_message_received(lines)


class AsynchronousLauncher(threading.Thread):

    def __init__(self, launcher):
        super(AsynchronousLauncher, self).__init__()
        self.launcher = launcher

    def run(self):
        # explicitly create event loop when not running in main thread
        if not isinstance(threading.current_thread(), threading._MainThread):
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)

        self.launcher.launch()
