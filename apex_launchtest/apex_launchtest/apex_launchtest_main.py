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

import argparse
import logging
from importlib.machinery import SourceFileLoader
import os
import sys

from ros2launch.api import print_arguments_of_launch_description

from .apex_runner import ApexRunner
from .junitxml import unittestResultsToXml

_logger_ = logging.getLogger(__name__)


def _load_python_file_as_module(python_file_path):
    """Load a given Python launch file (by path) as a Python module."""
    # Taken from apex_core to not introduce a weird dependency thing
    loader = SourceFileLoader('python_launch_file', python_file_path)
    return loader.load_module()


def apex_launchtest_main():

    logging.basicConfig()

    parser = argparse.ArgumentParser(
        description="Integration test framework for Apex AI"
    )

    parser.add_argument('test_file')

    parser.add_argument('-v', '--verbose',
                        action="store_true",
                        default=False,
                        help="Run with verbose output")

    parser.add_argument('-s', '--show-args', '--show-arguments',
                        action='store_true',
                        default=False,
                        help='Show arguments that may be given to the test file.')

    parser.add_argument(
        'launch_arguments',
        nargs='*',
        help="Arguments to the launch file; '<name>:=<value>' (for duplicates, last one wins)"
    )

    parser.add_argument(
        "--junit-xml",
        action="store",
        dest="xmlpath",
        default=None,
        help="write junit XML style report to specified path"
    )

    args = parser.parse_args()

    if args.verbose:
        _logger_.setLevel(logging.DEBUG)
        _logger_.debug("Running with verbose output")

    # Load the test file as a module and make sure it has the required
    # components to run it as an apex integration test
    _logger_.debug("Loading tests from file '{}'".format(args.test_file))
    if not os.path.isfile(args.test_file):
        # Note to future reader: parser.error also exits as a side effect
        parser.error("Test file '{}' does not exist".format(args.test_file))

    args.test_file = os.path.abspath(args.test_file)
    test_module = _load_python_file_as_module(args.test_file)

    _logger_.debug("Checking for generate_test_description")
    if not hasattr(test_module, 'generate_test_description'):
        parser.error(
            "Test file '{}' is missing generate_test_description function".format(args.test_file)
        )

    dut_test_description_func = test_module.generate_test_description
    _logger_.debug("Checking generate_test_description function signature")

    runner = ApexRunner(
        gen_launch_description_fn=dut_test_description_func,
        test_module=test_module,
        launch_file_arguments=args.launch_arguments,
        debug=args.verbose
    )

    _logger_.debug("Validating test configuration")
    try:
        runner.validate()
    except Exception as e:
        parser.error(e)

    if args.show_args:
        print_arguments_of_launch_description(
            launch_description=runner.get_launch_description()
        )
        sys.exit(0)

    _logger_.debug("Running integration test")
    try:
        result, postcheck_result = runner.run()
        _logger_.debug("Done running integration test")

        if args.xmlpath:
            xml_report = unittestResultsToXml(
                test_results={
                    "active_tests": result,
                    "after_shutdown_tests": postcheck_result
                }
            )
            xml_report.write(args.xmlpath, xml_declaration=True)

        if not result.wasSuccessful():
            sys.exit(1)

        if not postcheck_result.wasSuccessful():
            sys.exit(1)

    except Exception as e:
        import traceback
        traceback.print_exc()
        parser.error(e)
