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
from typing import Union

from launch.action import Action
from launch.actions import EmitEvent
from launch.actions import ExecuteProcess
from launch.actions import TimerAction
from launch.events import matches_action
from launch.events.process import SignalProcess
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType


class Test(ExecuteProcess):
    """Action that runs a test."""

    def __init__(
        self,
        *,
        timeout: Optional[Union[float, SomeSubstitutionsType]] = None,
        **kwargs
    ) -> None:
        """
        Constructor.

        Many arguments are passed to :class:`launch.ExecuteProcess`, so
        see the documentation for the class for additional details.

        :param: timeout the test will be killed after timeout seconds.
        """
        super().__init__(**kwargs)
        self.__timeout = timeout

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
        # Setup a timer to send us a SIGKILL if the test locks
        sigkill_timer = TimerAction(period=self.__timeout, actions=[
            EmitEvent(event=SignalProcess(
                signal_number=signal.SIGKILL,
                process_matcher=matches_action(self)
            )),
        ])
        if not actions:
            return [sigkill_timer]

        return actions.append(sigkill_timer)
