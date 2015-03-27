from launch.output_handler import CompositeOutputHandler
from launch.output_handler import ConsoleOutput
from launch.exit_handler import default_exit_handler


class LaunchDescriptor(object):

    def __init__(self):
        self.task_descriptors = []

    def add_coroutine(self, coroutine, name=None, exit_handler=None):
        if name is not None and name in [p.name for p in self.task_descriptors]:
            raise RuntimeError("Task name '%s' already used" % name)
        if exit_handler is None:
            exit_handler = default_exit_handler
        self.task_descriptors.append(CoroutineDescriptor(
            coroutine, name, exit_handler))

    def add_process(self, cmd, name=None, env=None, output_handlers=None, exit_handler=None):
        if name is not None and name in [p.name for p in self.task_descriptors]:
            raise RuntimeError("Task name '%s' already used" % name)
        if output_handlers is None:
            output_handlers = [ConsoleOutput()]
        output_handlers = CompositeOutputHandler(output_handlers)
        if exit_handler is None:
            exit_handler = default_exit_handler
        self.task_descriptors.append(ProcessDescriptor(
            cmd, name, output_handlers, exit_handler, env=env))


class TaskDescriptor(object):

    def __init__(self):
        self.task_state = None


class CoroutineDescriptor(TaskDescriptor):

    def __init__(self, coroutine, name, exit_handler):
        super(CoroutineDescriptor, self).__init__()
        self.coroutine = coroutine
        self.name = name
        self.exit_handler = exit_handler


class ProcessDescriptor(TaskDescriptor):

    def __init__(self, cmd, name, output_handler, exit_handler, env=None):
        super(ProcessDescriptor, self).__init__()
        self.cmd = cmd
        self.name = name
        self.output_handler = output_handler
        self.exit_handler = exit_handler
        self.env = env
        self.transport = None
        self.protocol = None
