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

import random
import socket

# To coordinate ROS_DOMAIN_IDs between multiple instances of apex_launchtest, we
# open a high numbered port "PORT_BASE + ROS_DOMAIN_ID"  If apex_launchtest manages to
# open the port, it can use that ROS_DOMAIN_ID for the duration of the test run
_PORT_BASE = 22119  # I picked this randomly as a high port that probably won't be in use


class _sockwrapper():
    """Wraps sockets to keep them open, but appear like a number from 1 to 100."""

    def __init__(self, socket):
        self.__socket = socket

    def __str__(self):
        return str(self.__socket.getsockname()[1] - _PORT_BASE)


class _default_selector:
    # By default, when we try to get a unique domain ID we'll start with a random
    # value, then increment from there until we find one that isn't taken

    def __init__(self):
        self._value = random.randint(1, 100)

    def __call__(self):
        retval = ((self._value - 1) % 100) + 1
        self._value += 1
        return retval


def get_coordinated_domain_id(*, selector=None):
    """
    Get a ROS_DOMAIN_ID from 1 to 100 that will not conflict with other ROS_DOMAIN_IDs.

    Other instances of apex_launchtest will use this same function to generate ROS_DOMAIN_IDs
    so that no two runs of apex_launchtest on the same system should conflict with one-another
    by default.  This is similar to the ROS1 rostest behavior of putting the ROS master
    on a unique port
    """
    if selector is None:
        selector = _default_selector()

    # Try 100 times to get a unique ROS domain ID
    for attempt in range(100):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            s.bind(('', _PORT_BASE + selector()))
        except OSError:
            continue
        else:
            return _sockwrapper(s)
    else:
        raise Exception('Failed to get a unique domain ID')
