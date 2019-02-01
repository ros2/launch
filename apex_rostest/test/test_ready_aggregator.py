# Copyright 2019 Apex.AI, Inc.
# All rights reserved.

from apex_rostest import ReadyAggregator

import unittest


class TestReadyAggregator(unittest.TestCase):

    def setUp(self):
        self.called = 0

    def parent_ready_fn(self):
        self.called += 1

    def test_aggregate_one(self):
        dut = ReadyAggregator(self.parent_ready_fn, 1)

        self.assertEqual(self.called, 0)
        dut.ready_fn()
        self.assertEqual(self.called, 1)

        # Make sure subsequent calls don't trigger the parent function
        dut.ready_fn()
        self.assertEqual(self.called, 1)

    def test_aggregate_multiple(self):
        NUM_CALLS = 10  # Maybe make this random?  Probably not worth the effort

        dut = ReadyAggregator(self.parent_ready_fn, NUM_CALLS)

        self.assertEqual(self.called, 0)

        for _ in range(9):
            dut.ready_fn()

        self.assertEqual(self.called, 0)
        dut.ready_fn()
        self.assertEqual(self.called, 1)

        # Make sure subsequent calls don't trigger the parent function
        dut.ready_fn()
        self.assertEqual(self.called, 1)
