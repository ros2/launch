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

"""Test the abstract Parser class."""

from unittest.mock import patch
import warnings  # noqa: F401

from launch.frontend.parser import importlib_metadata
from launch.frontend.parser import Parser


class InvalidEntryPoint:

    name = 'RefusesToLoad'

    def load(self):
        raise ValueError('I dont want to load!')


def test_invalid_launch_extension():
    with patch('warnings.warn') as mock_warn, \
            patch(importlib_metadata.__name__ + '.entry_points') as mock_ep:
        mock_ep.return_value = {
            'launch.frontend.launch_extension': [InvalidEntryPoint()]
        }

        Parser.load_launch_extensions()

        assert mock_ep.called
        assert mock_warn.call_args
        assert mock_warn.call_args[0][0].startswith('Failed to load')


def test_invalid_parser_implementations():
    with patch('warnings.warn') as mock_warn, \
            patch(importlib_metadata.__name__ + '.entry_points') as mock_ep:
        mock_ep.return_value = {
            'launch.frontend.parser': [InvalidEntryPoint()]
        }

        Parser.load_parser_implementations()

        assert mock_ep.called
        assert mock_warn.call_args
        assert mock_warn.call_args[0][0].startswith('Failed to load')
