class ExitHandler(object):

    """Interface for an exit handler."""

    def __init__(self):
        pass

    def should_restart(self, returncode):
        """Determine if the process should be restarted."""
        raise NotImplemented

    def should_tear_down(self, returncode):
        """Determine if all other processes should be terminated."""
        raise NotImplemented


class DefaultExitHandler(ExitHandler):

    """Terminate all other processes if this process exits."""

    def __init__(self):
        super(DefaultExitHandler, self).__init__()

    def should_restart(self, returncode):
        return False

    def should_tear_down(self, returncode):
        return True


class IgnoreExitHandler(ExitHandler):

    """Continue all other processes if this process exits."""

    def __init__(self):
        super(IgnoreExitHandler, self).__init__()

    def should_restart(self, returncode):
        return False

    def should_tear_down(self, returncode):
        return False


class RestartExitHandler(ExitHandler):

    """Restart this process if it exits."""

    def __init__(self):
        super(RestartExitHandler, self).__init__()

    def should_restart(self, returncode):
        return True

    def should_tear_down(self, returncode):
        return False
