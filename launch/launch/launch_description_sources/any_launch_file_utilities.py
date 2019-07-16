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

"""Python package utility functions related to loading Frontend Launch Files."""

from typing import Text
from typing import Type

from .frontend_launch_file_utilities import get_launch_description_from_frontend_launch_file
from .frontend_launch_file_utilities import InvalidFrontendLaunchFileError
from .python_launch_file_utilities import get_launch_description_from_python_launch_file
from .python_launch_file_utilities import InvalidPythonLaunchFileError
from ..frontend import Parser
from ..launch_description import LaunchDescription


class InvalidLaunchFileError(Exception):
    """Exception raised when the given launch file is not valid."""

    ...


def get_launch_description_from_any_launch_file(
    launch_file_path: Text,
    *,
    parser: Type[Parser] = Parser
) -> LaunchDescription:
    """
    Load a given launch file (by path), and return the launch description from it.

    :raise `InvalidLaunchFileError`: Failed to load launch file.
        It's only showed with launch files without extension (or not recognized extensions).
    :raise `SyntaxError`: Invalid file. The file may have a syntax error in it.
    :raise `ValueError`: Invalid file. The file may not be a text file.
    """
    try:
        extension = launch_file_path.rsplit('.', 1)[1]
    except IndexError:
        extension = None
    ex_to_show = None
    tried_frontend = False
    if extension == 'py':
        try:
            return get_launch_description_from_python_launch_file(launch_file_path)
        except Exception as ex:
            ex_to_show = ex
    elif Parser.is_extension_valid(extension):
        tried_frontend = True
        try:
            return get_launch_description_from_frontend_launch_file(launch_file_path)
        except InvalidLaunchFileError:
            pass
        except Exception as ex:
            ex_to_show = ex
    try:
        return get_launch_description_from_python_launch_file(launch_file_path)
    except (InvalidPythonLaunchFileError, SyntaxError):
        pass
    if not tried_frontend:
        try:
            return get_launch_description_from_frontend_launch_file(launch_file_path)
        except Exception:
            pass
    if ex_to_show is None:
        raise InvalidLaunchFileError('Failed to load launch file')
    else:
        raise ex_to_show
