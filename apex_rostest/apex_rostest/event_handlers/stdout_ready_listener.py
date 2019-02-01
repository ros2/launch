# Copyright 2019 Apex.AI, Inc.
# All rights reserved.

from launch.event_handlers import OnProcessIO
from launch.some_actions_type import SomeActionsType


class StdoutReadyListener(OnProcessIO):
    """
    Part of a LaunchDescription that can wait for nodes to signal ready with stdout.

    Some nodes signal that they're ready by printing a message to stdout.  This listener
    can be added to a launch description to wait for a particular node to output a particular
    bit of text
    """

    def __init__(self,
                 node_name,
                 ready_txt,
                 actions: [SomeActionsType]):
        self.__node_name = node_name
        self.__ready_txt = ready_txt
        self.__actions = actions

        super().__init__(
            on_stdout=self.__on_stdout
        )

    def __on_stdout(self, process_io):
        if self.__node_name in process_io.process_name:
            if self.__ready_txt in process_io.text.decode('ascii'):
                return self.__actions
