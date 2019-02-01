# Copyright 2019 Apex.AI, Inc.
# All rights reserved.

import unittest


def PreShutdownTestLoader():
    return _make_loader(False)


def PostShutdownTestLoader():
    return _make_loader(True)


def _make_loader(load_post_shutdown):

    class _loader(unittest.TestLoader):
        """TestLoader selectively loads pre-shutdown or post-shutdown tests."""

        def loadTestsFromTestCase(self, testCaseClass):

            if getattr(testCaseClass, "__post_shutdown_test__", False) == load_post_shutdown:
                return super(_loader, self).loadTestsFromTestCase(testCaseClass)
            else:
                # Empty test suites will be ignored by the test runner
                return self.suiteClass()

    return _loader()
