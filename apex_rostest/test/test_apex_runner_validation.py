# Copyright 2018 Apex.AI, Inc.
# All rights reserved.

import unittest

from apex_rostest import ApexRunner


class TestApexRunnerValidation(unittest.TestCase):

    def test_catches_bad_signature(self):

        dut = ApexRunner(
            gen_launch_description_fn=lambda: None,
            test_module=None
        )

        with self.assertRaises(TypeError):
            dut.validate()

        dut = ApexRunner(
            gen_launch_description_fn=lambda fn: None,
            test_module=None
        )

        dut.validate()
