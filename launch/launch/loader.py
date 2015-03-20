from importlib.machinery import SourceFileLoader
from launch import LaunchDescriptor


def load_launch_file(launch_file):
    launch_descriptor = LaunchDescriptor()

    loader = SourceFileLoader('temp', launch_file)
    launch_module = loader.load_module()
    launch_module.launch(launch_descriptor)

    return launch_descriptor
