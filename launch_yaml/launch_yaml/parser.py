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

"""Module for YAML Parser class."""

import io
from typing import Union

from launch import frontend

import yaml

from .entity import Entity


class Parser(frontend.Parser):
    """YAML parser implementation."""

    @classmethod
    def load(
        cls,
        stream: Union[str, io.TextIOBase],
    ) -> (Entity, 'Parser'):
        """Return entity loaded from YAML file."""
        if isinstance(stream, str):
            stream = open(stream, 'r')
        tree = yaml.safe_load(stream)
        if len(tree) != 1:
            raise RuntimeError('Expected only one root')
        type_name = list(tree.keys())[0]
        return (Entity(tree[type_name], type_name), cls())
