# Copyright 2018 Apex.AI, Inc.
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
import threading
import unittest
import uuid

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
import rclpy

import std_msgs.msg

from apex_launchtest.event_handlers import StdoutReadyListener


def generate_test_description(ready_fn):
    node_env = os.environ.copy()
    node_env["PYTHONUNBUFFERED"] = "1"

    return LaunchDescription([
        Node(package='apex_launchtest', node_executable='echo_node', env=node_env),
        RegisterEventHandler(
            StdoutReadyListener(
                "echo_node",
                "ready",
                actions=[OpaqueFunction(function=lambda context: ready_fn())]
            )
        )
    ])


class MessageListener:

    def __init__(self):
        self.msg = None
        self.event = threading.Event()

    def callback(self, msg):
        print("got msg {}".format(msg.data))
        self.msg = msg
        self.event.set()


class TestEcho(unittest.TestCase):

    def test_node_echo(self):

        msg_listener = MessageListener()

        # Publish to the 'echo' node on the 'listen' topic and look for a response on the
        # echo topic
        pub = self.node.create_publisher(
            msg_type=std_msgs.msg.String,
            topic="listen"
        )

        self.node.create_subscription(
            msg_type=std_msgs.msg.String,
            topic="echo",
            callback=msg_listener.callback
        )

        msg = std_msgs.msg.String()
        msg.data = str(uuid.uuid4())

        pub.publish(msg)

        for _ in range(5):
            rclpy.spin_once(self.node, timeout_sec=1.0)
            if msg_listener.event.is_set():
                break

        self.assertTrue(msg_listener.event.wait(0),
                        "Timed out waiting for echo")

        self.assertEqual(msg.data, msg_listener.msg.data)
