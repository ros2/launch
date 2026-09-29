# Copyright 2025 Open Source Robotics Foundation, Inc.
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

from typing import TextIO
from typing import Tuple
from typing import Union

from launch.frontend import Parser
from launch.frontend.entity import Entity
from launch.utilities.typing_file_path import FilePath


def load_no_extensions(file: Union[FilePath, TextIO]) -> Tuple[Entity, 'Parser']:
    entity, parser = Parser.load(file)
    # By default, the parser returned from Parser.load() will attempt to load in all extensions.
    # However, those extensions tend to have other dependencies that *this* package doesn't
    # have (like rclpy).  Thus, attempting to load those in can fail.  Here we explicitly disable
    # the loading of extra extensions to avoid this problem.
    parser.__class__.extensions_loaded = True
    return entity, parser
