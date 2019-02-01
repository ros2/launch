# Copyright 2019 Apex.AI, Inc.
# All rights reserved.


from launch_ros.actions import Node as __rl_node


def EmptyNode():
    return __rl_node(package='apex_rostest', node_executable='empty_node')
