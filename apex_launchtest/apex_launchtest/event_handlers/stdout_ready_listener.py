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
