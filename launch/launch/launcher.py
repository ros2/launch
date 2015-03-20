import asyncio
import signal
from threading import Lock

from launch.protocol import SubprocessProtocol


class DefaultLauncher(object):

    def __init__(self, name_prefix=''):
        self.name_prefix = name_prefix
        self.process_descriptors = []
        self.print_mutex = Lock()

    def add_launch_descriptor(self, launch_descriptor):
        for process_descriptor in launch_descriptor.process_descriptors:
            # automatic naming if not specified
            if process_descriptor.name is None:
                name = str(len(self.process_descriptors))
                if name in [p.name for p in self.process_descriptors]:
                    raise RuntimeError("Process name '%s' already used" % name)
                process_descriptor.name = name

            self.process_descriptors.append(process_descriptor)

    def launch(self):
        loop = asyncio.get_event_loop()
        returncode = loop.run_until_complete(self._run(loop))
        loop.close()

        return returncode

    @asyncio.coroutine
    def _run(self, loop):
        # start all processes and collect their exit futures
        all_futures = {}
        for index, p in enumerate(self.process_descriptors):
            p.output_handler.set_print_mutex(self.print_mutex)
            p.output_handler.set_line_prefix('[%s] ' % p.name)

            yield from self._spawn_process(loop, index)
            all_futures[p.protocol.exit_future] = index

        rc = 0
        while True:
            # skip if no more processes to run
            if not all_futures:
                break

            # wait for any process to finish
            yield from asyncio.wait(all_futures.keys(), return_when=asyncio.FIRST_COMPLETED)

            # collect done processes
            done_indices = [index for future, index in all_futures.items() if future.done()]
            done_process_descriptors = [self.process_descriptors[index] for index in done_indices]

            # close transport, collect return code and remove future
            for p in done_process_descriptors:
                self._close_process(p)
                del all_futures[p.protocol.exit_future]

            # check if all processes should be stopped
            tear_down_returncodes = [
                p.returncode for p in done_process_descriptors
                if p.exit_handler.should_tear_down(p.returncode)]
            if tear_down_returncodes:
                with self.print_mutex:
                    print('() tear down')
                if len(tear_down_returncodes) == 1:
                    rc = tear_down_returncodes[0]
                else:
                    rc = 1 if any(tear_down_returncodes) else 0
                break

            # restart processes if requested
            restart_indices = [
                index for index in done_indices
                if self.process_descriptors[index].exit_handler.should_restart(
                    self.process_descriptors[index].returncode)
            ]
            for index in restart_indices:
                yield from self._spawn_process(loop, index)
                all_futures[self.process_descriptors[index].protocol.exit_future] = index

        # terminate all remaining processes
        if all_futures:

            # sending SIGINT to remaining processes
            for index in all_futures.values():
                p = self.process_descriptors[index]
                self._process_message(p, 'signal SIGINT')
                p.transport.send_signal(signal.SIGINT)

            yield from asyncio.wait(all_futures.keys(), timeout=3)

            # sending SIGINT to remaining processes
            for index in all_futures.values():
                p = self.process_descriptors[index]
                if not p.protocol.exit_future.done():
                    self._process_message(p, 'signal SIGTERM')
                    p.transport.send_signal(signal.SIGTERM)

            yield from asyncio.wait(all_futures.keys())

            # close all remaining processes
            for index in all_futures.values():
                p = self.process_descriptors[index]
                self._close_process(p)

        return rc

    def _spawn_process(self, loop, index):
        p = self.process_descriptors[index]
        p.output_handler.process_init()
        kwargs = {}
        if p.output_handler.support_stderr2stdout():
            kwargs['stderr'] = asyncio.subprocess.STDOUT
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
        p.returncode = p.transport.get_returncode()
        self._process_message(p, 'rc %d' % p.returncode)
        p.output_handler.process_cleanup()

    def _process_message(self, process_descriptor, message):
        p = process_descriptor

        with self.print_mutex:
            print('(%s)' % p.name, message)
        lines = (message + '\n').encode()
        p.output_handler.on_message_received(lines)
