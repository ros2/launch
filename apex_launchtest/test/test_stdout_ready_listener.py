# Copyright 2019 Apex.AI, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import unittest

import ament_index_python
import launch
import launch.actions

from apex_launchtest.event_handlers import StdoutReadyListener


class TestStdoutReadyListener(unittest.TestCase):

    def setUp(self):
        # Set up a launch description for the tests to use
        node_env = os.environ.copy()
        node_env["PYTHONUNBUFFERED"] = "1"

        self.launch_description = launch.LaunchDescription([
            launch.actions.ExecuteProcess(
                cmd=[
                    os.path.join(
                        ament_index_python.get_package_prefix('apex_launchtest'),
                        'lib/apex_launchtest',
                        'terminating_proc',
                    )
                ],
                env=node_env
            )
        ])

    def test_wait_for_ready(self):
        data = []

        self.launch_description.add_entity(
            launch.actions.RegisterEventHandler(
                StdoutReadyListener(
                    node_name="terminating_proc",
                    ready_txt="Ready",
                    actions=[
                        launch.actions.OpaqueFunction(function=lambda context: data.append('ok'))
                    ]
                )
            )
        )

        launch_service = launch.LaunchService()
        launch_service.include_launch_description(self.launch_description)
        launch_service.run()

        # If the StdoutReadyListener worked, we should see 'ok' in the data
        self.assertIn('ok', data)

    def test_wait_for_wrong_process(self):
        data = []

        self.launch_description.add_entity(
            launch.actions.RegisterEventHandler(
                StdoutReadyListener(
                    node_name="different_node",
                    ready_txt="Ready",
                    actions=[
                        launch.actions.OpaqueFunction(function=lambda context: data.append('ok'))
                    ]
                )
            )
        )

        launch_service = launch.LaunchService()
        launch_service.include_launch_description(self.launch_description)
        launch_service.run()

        # We should not get confused by output from another node
        self.assertNotIn('ok', data)

    def test_wait_for_wrong_message(self):
        data = []

        self.launch_description.add_entity(
            launch.actions.RegisterEventHandler(
                StdoutReadyListener(
                    node_name="different_node",
                    ready_txt="not_ready",
                    actions=[
                        launch.actions.OpaqueFunction(function=lambda context: data.append('ok'))
                    ]
                )
            )
        )

        launch_service = launch.LaunchService()
        launch_service.include_launch_description(self.launch_description)
        launch_service.run()

        # We should not get confused by output that doesn't match the ready_txt
        self.assertNotIn('ok', data)
