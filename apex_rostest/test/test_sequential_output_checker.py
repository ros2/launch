# Copyright 2019 Apex.AI, Inc.
# All rights reserved.

import unittest

from apex_rostest.asserts.assert_sequential_output import _SequentialOutputChecker


class TestAssertSequentialStdout(unittest.TestCase):

    def setUp(self):
        self.to_check = [
            "output 10",
            "output 15",
            "output 20",
            "multi-line 1\nmulti-line 2\nmulti-line 3",
            "aaaaa bbbbb ccccc ddddd eeeee fffff ggggg hhhhh iiiii jjjjj",  # long line
            "xxxxx yyyyy\nsome dummy text\nzzzzz",  # mix of long line and multi-line
            "output 20",
            "!@#$%^&*()",  # Help find off by one errors in the substring logic
        ]

        self.dut = _SequentialOutputChecker(self.to_check)

    def test_good_sequential_output(self):

        for output in self.to_check:
            self.dut.assertInStdout(output)

        with self.assertRaises(AssertionError):
            # This should assert because we've already moved past "output 10"
            self.dut.assertInStdout(self.to_check[0])

    def test_non_matching_output_does_not_advance_state(self):
        # Make sure we can match correct output even after failing to match something

        with self.assertRaises(AssertionError):
            self.dut.assertInStdout("bad output not found")

        self.test_good_sequential_output()

    def test_multi_line_find(self):
        self.dut.assertInStdout("multi-line 1")
        self.dut.assertInStdout("multi-line 2")
        self.dut.assertInStdout("multi-line 3")

        with self.assertRaises(AssertionError):
            self.dut.assertInStdout("multi-line 1")

    def test_long_line_find(self):
        self.dut.assertInStdout("ccccc")
        self.dut.assertInStdout("ddddd")
        self.dut.assertInStdout("eeeee")

        with self.assertRaises(AssertionError):
            self.dut.assertInStdout("aaaaa")

    def test_duplicates_advances_state(self):
        self.dut.assertInStdout("output 20")
        self.dut.assertInStdout("output 20")

        with self.assertRaises(AssertionError):
            self.dut.assertInStdout("multi-line 1")

    def test_individual_character_find(self):
        self.dut.assertInStdout("!")
        self.dut.assertInStdout("@")
        self.dut.assertInStdout("#")
        self.dut.assertInStdout("$")

        # Skip ahead
        self.dut.assertInStdout("*")
        self.dut.assertInStdout("(")

        # Check the same character
        with self.assertRaises(AssertionError):
            self.dut.assertInStdout("(")

    def test_mixed_multi_line(self):
        self.dut.assertInStdout("xxxxx")
        self.dut.assertInStdout("some dummy text")

        with self.assertRaises(AssertionError):
            self.dut.assertInStdout("yyyyy")
