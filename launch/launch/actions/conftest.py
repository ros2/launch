# imports needed for doctests
import launch
from launch import LaunchDescription
import launch.actions

import pytest


@pytest.fixture(autouse=True)
def add_np(doctest_namespace):
    doctest_namespace['launch'] = launch
    doctest_namespace['LaunchDescription'] = LaunchDescription
    for x in launch.actions.__all__:
        doctest_namespace[x] = getattr(launch.actions, x)
