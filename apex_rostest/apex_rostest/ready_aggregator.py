# Copyright 2019 Apex.AI, Inc.
# All rights reserved.

import threading


class ReadyAggregator:
    """Calls a ready_fn parent function on the nth call to a child function."""

    def __init__(self, ready_fn, num_to_aggregate):
        """Create a ReadyAggregator.

        :param callable ready_fn: The function to call after n calls to ReadyAggregator.ready_fn
        :param int num_to_aggregate: Number of calls to ReadyAggregator.ready_fn necessary for
        the parent ready_fn to be called
        """
        self._parent_ready_fn = ready_fn
        self._count_to_activate = num_to_aggregate

        self._lock = threading.Lock()

    def ready_fn(self):

        # We don't want to call the parent ready function while holding a lock
        # in case it does something that will try to acquire the same lock.  Instead,
        # we'll make a local copy of the count and evaluate it outside of the critical
        # section
        local_count = 0
        with self._lock:
            self._count_to_activate -= 1
            local_count = self._count_to_activate

        if local_count == 0:
            self._parent_ready_fn()
