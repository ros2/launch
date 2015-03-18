class ProcessDescriptor(object):

    def __init__(self, cmd, name, output_handler, exit_handler, env=None):
        self.cmd = cmd
        self.name = name
        self.output_handler = output_handler
        self.exit_handler = exit_handler
        self.env = env
        self.transport = None
        self.protocol = None
        self.returncode = None
