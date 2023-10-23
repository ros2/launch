# Copyright 2023 Metro Robots
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

"""Module for quickly making a required process."""

from typing import Optional

from ..actions import ExecuteProcess, RegisterEventHandler, LogInfo, EmitEvent
from ..events import Shutdown
from ..event_handlers import OnProcessExit
from ..some_substitutions_type import SomeSubstitutionsType


def shutdown_on_process_exit(process: ExecuteProcess,
                             msg: Optional[SomeSubstitutionsType] = None) -> RegisterEventHandler:
    exit_actions = []
    if msg is not None:
        exit_actions.append(LogInfo(msg=msg))
    exit_actions.append(EmitEvent(event=Shutdown()))

    event_handler = OnProcessExit(
        target_action=process,
        on_exit=exit_actions,
    )
    return RegisterEventHandler(event_handler=event_handler)
