# Copyright 2019 Apex.AI, Inc.
# All rights reserved.

import threading

import rclpy


class MessagePump:
    """Calls rclpy.spin on a thread so tests don't need to."""

    def __init__(self, node):
        self._node = node
        self._thread = threading.Thread(
            target=self._run,
            name="msg_pump_thread",
        )
        self._run = True

    def start(self):
        self._thread.start()

    def stop(self):
        self._run = False
        self._thread.join(timeout=5.0)
        if self._thread.is_alive():
            raise Exception("Timed out waiting for message pump to stop")

    def _run(self):
        while self._run:
            rclpy.spin_once(self._node, timeout_sec=1.0)
