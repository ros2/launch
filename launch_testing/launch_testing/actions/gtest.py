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

from typing import Text

from launch.substitutions import FindExecutable

from .test import Test


class GTest(Test):
    """Action that runs a GTest."""

    def __init__(
        self,
        *,
        path: Text,
        **kwargs
    ) -> None:
        """
        TODO(ivanpauno).

        Write documentation.
        """
        self.__path = path
        cmd = [FindExecutable(name=path)]
        super().__init__(cmd=cmd, **kwargs)

    @property
    def path(self):
        """Getter for path."""
        return self.__path
