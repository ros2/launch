# Copyright 2019 Apex.AI, Inc.
# All rights reserved.

import os
import unittest

from launch import LaunchService
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessIO
from launch_ros.actions import Node

from apex_rostest import ActiveIoHandler
from apex_rostest.asserts import assertInStdout
from apex_rostest.asserts import NO_CMD_ARGS


class TestIoHandlerAndAssertions(unittest.TestCase):

    EXPECTED_TEXT = "Ready"  # Expected to be in every test run
    NOT_FOUND_TEXT = "Zazzlefraz"  # Not expected to be in the output anywhere

    @classmethod
    def setUpClass(cls):
        # It's easier to actually capture some IO from the launch system than it is to fake it
        # but it takes a few seconds.  We'll do it once and run tests on the same captured
        # IO

        node_env = os.environ.copy()
        node_env["PYTHONUNBUFFERED"] = "1"

        cls.proc_output = ActiveIoHandler()

        cls.node_1 = Node(package='apex_rostest',
                          node_executable='terminating_node',
                          env=node_env)

        # This node should be distinguishable by its cmd line args
        cls.node_2 = Node(package='apex_rostest',
                          node_executable='terminating_node',
                          arguments=['--extra'],
                          env=node_env)

        # This node should be distinguishable by its diffetent node name
        cls.node_3 = Node(package='apex_rostest',
                          node_executable='terminating_node',
                          node_name='different_name',
                          env=node_env)

        launch_description = LaunchDescription([
            cls.node_1,
            cls.node_2,
            cls.node_3,
            # This plumbs all the output to our IoHandler just like the ApexRunner does
            RegisterEventHandler(
                OnProcessIO(
                    on_stdout=cls.proc_output.append,
                    on_stderr=cls.proc_output.append,
                )
            )
        ])

        launch_service = LaunchService()
        launch_service.include_launch_description(launch_description)
        launch_service.run()

    def test_all_processes_had_io(self):
        # Should have three nodes (processes)
        self.assertEqual(3, len(self.proc_output.processes()))

    def test_only_one_process_had_arguments(self):
        text_lines = [t.text.decode('ascii') for t in self.proc_output]
        print("All text: {}".format(text_lines))

        matches = [t for t in text_lines if "Called with arguments" in t]
        print("Called with arguments: {}".format(matches))

        # Two process have args, because thats how node names are passed down
        self.assertEqual(2, len(matches))

        matches_extra = [t for t in matches if "--extra" in t]
        self.assertEqual(1, len(matches_extra))

        matches_node_name = [t for t in matches if "node:=different_name" in t]
        self.assertEqual(1, len(matches_node_name))

    def test_assert_wait_for_returns_immediately(self):
        # If the output has already been seen, ensure that assertWaitsFor returns right away
        self.proc_output.assertWaitFor("Emulating Setup", timeout=1)

    def test_EXPECTED_TEXT_is_present(self):
        # Sanity check - makes sure the EXPECTED_TEXT is somewhere in the test run
        text_lines = [t.text.decode('ascii') for t in self.proc_output]
        contains_ready = [self.EXPECTED_TEXT in t for t in text_lines]
        self.assertTrue(any(contains_ready))

    def test_process_names(self):
        self.assertIn("terminating_node-1", self.proc_output.process_names())
        self.assertIn("terminating_node-2", self.proc_output.process_names())
        self.assertIn("terminating_node-3", self.proc_output.process_names())

    def test_processes(self):
        self.assertIn(self.node_1, self.proc_output.processes())
        self.assertIn(self.node_2, self.proc_output.processes())
        self.assertIn(self.node_3, self.proc_output.processes())

    # ---------- Tests for assertInStdout below this line ----------
    def test_assertInStdout_notices_no_matching_node(self):
        with self.assertRaisesRegex(Exception, "Did not find any process") as cm:
            assertInStdout(self.proc_output, self.EXPECTED_TEXT, "bad_node_name")

        print(cm.exception)

        # Make sure the assertion method lists the names of the process it does have:
        self.assertIn("terminating_node-1", str(cm.exception))
        self.assertIn("terminating_node-2", str(cm.exception))
        self.assertIn("terminating_node-3", str(cm.exception))

    def test_assertInStdout_notices_too_many_matching_nodes(self):
        with self.assertRaisesRegex(Exception, "Found multiple processes") as cm:
            assertInStdout(self.proc_output, self.EXPECTED_TEXT, "terminating_node")

        # Make sure the assertion method lists the names of the duplicate procs:
        self.assertIn("terminating_node-1", str(cm.exception))
        self.assertIn("terminating_node-2", str(cm.exception))
        self.assertIn("terminating_node-3", str(cm.exception))

    def test_strict_node_matching_false(self):
        assertInStdout(
            self.proc_output,
            self.EXPECTED_TEXT,
            "terminating_node",
            strict_node_matching=False
        )

    def test_arguments_disambiguate_processes(self):
        txt = self.EXPECTED_TEXT
        assertInStdout(self.proc_output, txt, "terminating_node", "--extra")
        assertInStdout(self.proc_output, txt, "terminating_node", "__node:=different_name")
        assertInStdout(self.proc_output, txt, "terminating_node", NO_CMD_ARGS)

    def test_asserts_on_missing_text(self):
        with self.assertRaises(AssertionError):
            assertInStdout(self.proc_output, self.NOT_FOUND_TEXT, "terminating", NO_CMD_ARGS)
