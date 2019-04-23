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

"""Module for Entity class."""

import io
from typing import Any
from typing import List
from typing import Optional
from typing import Text
from typing import Union

from launch.utilities import is_a

from pkg_resources import iter_entry_points

frontend_entities = None


class Entity:
    """Single item in the intermediate front_end representation."""

    @staticmethod
    def load(
        file: Union[str, io.TextIOBase],
        parent: 'Entity' = None
    ) -> 'Entity':
        """Return an entity loaded with a markup file."""
        # frontend_entities is not global to avoid a recursive import
        global frontend_entities
        if frontend_entities is None:
            frontend_entities = {
                entry_point.name: entry_point.load()
                for entry_point in iter_entry_points('launch_frontend.entity')
            }
        if is_a(file, str):
            # This automatically recognizes 'file.xml' or 'file.launch.xml'
            # as a launch file using the xml frontend.
            frontend_name = file.rsplit('.', 1)[1]
            if frontend_name in frontend_entities:
                return frontend_entities[frontend_name].load(file)
        # If not, apply brute force.
        # TODO(ivanpauno): Maybe, we want to force correct file naming.
        # In that case, we should raise an error here.
        # Note(ivanpauno): It's impossible to recognize a wrong formatted
        # file from a non-recognized front-end implementation.
        for implementation in frontend_entities:
            try:
                return implementation.load(file)
            except Exception:
                pass
        raise RuntimeError('Not recognized front-end implementation.')

    @staticmethod
    def frontend() -> Text:
        """Get which frontend is wrapping."""
        raise NotImplementedError()

    @property
    def type_name(self) -> Text:
        """Get Entity type."""
        raise NotImplementedError()

    @property
    def parent(self) -> Optional['Entity']:
        """Get Entity parent."""
        raise NotImplementedError()

    @property
    def children(self) -> Optional[List['Entity']]:
        """Get Entity children."""
        raise NotImplementedError()

    def __getattr__(self, name: Text) -> Optional[Any]:
        """Get attribute."""
        raise NotImplementedError()
