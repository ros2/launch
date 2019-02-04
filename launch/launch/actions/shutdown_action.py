# Copyright 2019 Open Source Robotics Foundation, Inc.
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

"""Module for the Shutdown action."""

import logging

from ..action import Action
from ..events import Shutdown as ShutdownEvent
from ..events.process import ProcessExited
from ..launch_context import LaunchContext

_logger = logging.getLogger(name='launch')


class Shutdown(Action):
    """Action that shuts down a launched system by emitting Shutdown when executed."""

    def execute(self, context: LaunchContext):
        """Execute the action."""
        try:
            event = context.locals.event
        except AttributeError:
            event = None

        if isinstance(event, ProcessExited):
            _logger.info('process[{}] was required: shutting down launched system'.format(
                event.process_name))

        context.emit_event_sync(ShutdownEvent())
