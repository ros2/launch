# import pytest
from launch.descriptions.executable import Executable
from launch.launch_context import LaunchContext


def test_executable():
    exe = Executable(cmd="test")
    assert exe is not None


def test_cmd_simple_string():
    exe = Executable(cmd='ls "my/subdir/with spaces/"')
    exe.apply_context(LaunchContext())
    assert all([a == b for a, b in zip(exe.final_cmd, ['ls', 'my/subdir/with spaces/'])])


def test_cmd_string_in_list():
    exe = Executable(cmd=['ls "my/subdir/with spaces/"'])
    exe.apply_context(LaunchContext())
    assert all([a == b for a, b in zip(exe.final_cmd, ['ls', 'my/subdir/with spaces/'])])


def test_cmd_strings_in_list():
    exe = Executable(cmd=['ls', '"my/subdir/with spaces/"'])
    exe.apply_context(LaunchContext())
    assert all([a == b for a, b in zip(exe.final_cmd, ['ls', 'my/subdir/with spaces/'])])


def test_cmd_multiple_arguments_in_string():
    exe = Executable(cmd=['ls', '-opt1', '-opt2', '-opt3'])
    exe.apply_context(LaunchContext())
    assert all([a == b for a, b in zip(exe.final_cmd, ['ls', '-opt1', '-opt2', '-opt3'])])
