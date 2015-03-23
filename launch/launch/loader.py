from importlib.machinery import SourceFileLoader


def load_launch_file(launch_file, launch_descriptor, argv):
    loader = SourceFileLoader('launch_file', launch_file)
    launch_module = loader.load_module()
    launch_module.launch(launch_descriptor, argv)
