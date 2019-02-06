# Copyright 2018 Open Source Robotics Foundation, Inc.
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

"""Module for the GTest action."""

import signal
from typing import List
from typing import Optional
from typing import Text

from launch.action import Action
from launch.actions import EmitEvent
from launch.actions import ExecuteProcess
from launch.actions import TimerAction
from launch.events.process import matches_action
from launch.events.process import SignalProcess

from launch.launch_context import LaunchContext


class GTest(ExecuteProcess):
    """Action that runs a GTest."""

    def __init__(
        self,
        *,
        path: Text,
        timeout: Optional[float] = None,
        **kwargs
    ) -> None:
        """
        TODO(ivanpauno) Write documentation
        """
        # cmd += [] if arguments is None else arguments
        super().__init__(cmd=path, **kwargs)
        self.__path = path
        self.__timeout = timeout
        # self.__substitutions_performed = False

    @property
    def path(self):
        """Getter for path."""
        return self.__path

    @property
    def timeout(self):
        """Getter for timeout."""
        return self.__timeout

    def execute(self, context: LaunchContext) -> Optional[List[Action]]:
        """
        Execute the action.

        Delegated to :meth:`launch.actions.ExecuteProcess.execute`.
        """
        actions = super().execute(context)

        if not self.__timeout:
            return actions

        # Setup a timer to send us a SIGTERM if we don't shutdown quickly.
        sigkill_timer = TimerAction(period=self.__timeout, actions=[
            EmitEvent(event=SignalProcess(
                signal_number=signal.SIGKILL,
                process_matcher=matches_action(self)
            )),
        ])
        return actions.append(sigkill_timer)
