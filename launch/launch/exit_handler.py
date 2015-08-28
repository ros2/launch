# Copyright 2015 Open Source Robotics Foundation, Inc.
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

class ExitHandlerContext(object):

    """The context which is passed to an exit handler function."""

    def __init__(self, launch_state, task_state):
        self.launch_state = launch_state
        self.task_state = task_state


def default_exit_handler(context):
    """
    Trigger teardown of launch.

    Use the returncode of the task for the launch if the launch was not already tearing down.
    """
    # trigger tear down if not already tearing down
    if not context.launch_state.teardown:
        context.launch_state.teardown = True

    # set launch return code if not already set
    if not context.launch_state.returncode:
        try:
            rc = int(context.task_state.returncode)
        except (TypeError, ValueError):
            rc = 1 if bool(context.task_state.returncode) else 0
        context.launch_state.returncode = rc


def ignore_exit_handler(context):
    """Continue the launch and don't affect the returncode of the launch."""
    pass


def restart_exit_handler(context):
    """Request restart of the task."""
    context.task_state.restart = True


def primary_exit_handler(context):
    """
    Trigger teardown of launch and if teardown already in place set non-zero return code.

    Same as default exit handler but if teardown was triggered by another task
    ensure that the returncode is non-zero.
    """
    if context.launch_state.teardown:
        if not context.launch_state.returncode:
            context.launch_state.returncode = 1

    default_exit_handler(context)
