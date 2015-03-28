class TaskState(object):

    def __init__(self):
        self.argv = []
        self.exception = None
        self.restart = False
        self.restart_count = 0
        self.returncode = None
