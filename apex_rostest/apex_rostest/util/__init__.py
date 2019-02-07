# Copyright 2019 Apex.AI, Inc.
# All rights reserved.


from launch_ros.actions import Node as __rl_node
from .message_pump import MessagePump


def EmptyNode():
    return __rl_node(package='apex_rostest', node_executable='empty_node')


__all__ = [
    'EmptyNode',
    'MessagePump',
]
