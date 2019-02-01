# Copyright 2019 Apex.AI, Inc.
# All rights reserved.

import os
import unittest

from launch import LaunchService
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

from apex_rostest.event_handlers import StdoutReadyListener


class TestStdoutReadyListener(unittest.TestCase):

    def setUp(self):
        # Set up a launch description for the tests to use
        node_env = os.environ.copy()
        node_env["PYTHONUNBUFFERED"] = "1"

        self.launch_description = LaunchDescription([
            Node(package='apex_rostest',
                 node_executable='terminating_node',
                 env=node_env)
        ])

    def test_wait_for_ready(self):
        data = []

        self.launch_description.add_entity(
            RegisterEventHandler(
                StdoutReadyListener(
                    node_name="terminating_node",
                    ready_txt="Ready",
                    actions=[OpaqueFunction(function=lambda context: data.append('ok'))]
                )
            )
        )

        launch_service = LaunchService()
        launch_service.include_launch_description(self.launch_description)
        launch_service.run()

        # If the StdoutReadyListener worked, we should see 'ok' in the data
        self.assertIn('ok', data)

    def test_wait_for_wrong_process(self):
        data = []

        self.launch_description.add_entity(
            RegisterEventHandler(
                StdoutReadyListener(
                    node_name="different_node",
                    ready_txt="Ready",
                    actions=[OpaqueFunction(function=lambda context: data.append('ok'))]
                )
            )
        )

        launch_service = LaunchService()
        launch_service.include_launch_description(self.launch_description)
        launch_service.run()

        # We should not get confused by output from another node
        self.assertNotIn('ok', data)

    def test_wait_for_wrong_message(self):
        data = []

        self.launch_description.add_entity(
            RegisterEventHandler(
                StdoutReadyListener(
                    node_name="different_node",
                    ready_txt="not_ready",
                    actions=[OpaqueFunction(function=lambda context: data.append('ok'))]
                )
            )
        )

        launch_service = LaunchService()
        launch_service.include_launch_description(self.launch_description)
        launch_service.run()

        # We should not get confused by output that doesn't match the ready_txt
        self.assertNotIn('ok', data)
