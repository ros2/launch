import asyncio


class SubprocessProtocol(asyncio.SubprocessProtocol):

    def __init__(self, output_handler, *args, **kwargs):
        self.output_handler = output_handler
        self.exit_future = asyncio.Future()

        self.stdin = None
        self.stdout = None
        self.stderr = None

        asyncio.SubprocessProtocol.__init__(self, *args, **kwargs)

    def connection_made(self, transport):
        self.stdin = transport.get_pipe_transport(0)
        self.stdout = transport.get_pipe_transport(1)
        self.stderr = transport.get_pipe_transport(2)

    def pipe_data_received(self, fd, data):
        # This function is only called when pty's are not being used
        stdout = self.stdout
        if not isinstance(stdout, int):
            stdout = 1
        if fd == stdout:
            if hasattr(self, 'on_stdout_received'):
                self.on_stdout_received(data)
        else:
            assert fd == 2
            if hasattr(self, 'on_stderr_received'):
                self.on_stderr_received(data)

    def on_stdout_received(self, data):
        self.output_handler.on_stdout_received(data)

    def on_stderr_received(self, data):
        self.output_handler.on_stderr_received(data)

    def process_exited(self):
        self.output_handler.flush()
        self.exit_future.set_result(0)
