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
from importlib.machinery import SourceFileLoader
import logging
import os
import sys

from .domain_coordinator import get_coordinated_domain_id
from .junitxml import unittestResultsToXml
from .loader import LoadTestsFromPythonModule
from .print_arguments import print_arguments_of_launch_description
from .test_runner import LaunchTestRunner

_logger_ = logging.getLogger(__name__)


def _load_python_file_as_module(test_module_name, python_file_path):
    """Load a given Python launch file (by path) as a Python module."""
    # Taken from launch_testing to not introduce a weird dependency thing
    loader = SourceFileLoader(test_module_name, python_file_path)
    return loader.load_module()


def main():

    logging.basicConfig()

    parser = argparse.ArgumentParser(
        description='Launch integration testing tool'
    )

    parser.add_argument('test_file')

    parser.add_argument('-v', '--verbose',
                        action='store_true',
                        default=False,
                        help='Run with verbose output')

    parser.add_argument('-s', '--show-args', '--show-arguments',
                        action='store_true',
                        default=False,
                        help='Show arguments that may be given to the test file.')

    # TODO(hidmic): Provide this option for rostests only.
    parser.add_argument('-i', '--isolated',
                        action='store_true',
                        default=False,
                        help=('Isolate tests using a custom ROS_DOMAIN_ID.'
                              'Useful for test parallelization.'))

    parser.add_argument(
        'launch_arguments',
        nargs='*',
        help="Arguments to the launch file; '<name>:=<value>' (for duplicates, last one wins)"
    )

    parser.add_argument(
        '--junit-xml',
        action='store',
        dest='xmlpath',
        default=None,
        help='write junit XML style report to specified path'
    )

    parser.add_argument(
        '--package-name',
        action='store',
        default=None,
        help='a name for the test'
    )
    args = parser.parse_args()

    if args.verbose:
        _logger_.setLevel(logging.DEBUG)
        _logger_.debug('Running with verbose output')

    if args.isolated:
        domain_id = get_coordinated_domain_id()  # Must copy this to a local to keep it alive
        _logger_.debug('Running with ROS_DOMAIN_ID {}'.format(domain_id))
        os.environ['ROS_DOMAIN_ID'] = str(domain_id)

    # Load the test file as a module and make sure it has the required
    # components to run it as a launch test
    _logger_.debug("Loading tests from file '{}'".format(args.test_file))
    if not os.path.isfile(args.test_file):
        # Note to future reader: parser.error also exits as a side effect
        parser.error("Test file '{}' does not exist".format(args.test_file))

    args.test_file = os.path.abspath(args.test_file)
    test_file_basename = os.path.splitext(os.path.basename(args.test_file))[0]
    if not args.package_name:
        args.package_name = test_file_basename
    test_module = _load_python_file_as_module(args.package_name, args.test_file)

    _logger_.debug('Checking for generate_test_description')
    if not hasattr(test_module, 'generate_test_description'):
        parser.error(
            "Test file '{}' is missing generate_test_description function".format(args.test_file)
        )

    # This is a list of TestRun objects.  Each run corresponds to one launch.  There may be
    # multiple runs if the launch is parametrized
    test_runs = LoadTestsFromPythonModule(
        test_module, name='{}.{}.launch_tests'.format(
            args.package_name, test_file_basename
        )
    )

    # The runner handles sequcing the launches
    runner = LaunchTestRunner(
        test_runs=test_runs,
        launch_file_arguments=args.launch_arguments,
        debug=args.verbose
    )

    _logger_.debug('Validating test configuration')
    try:
        runner.validate()
    except Exception as e:
        parser.error(e)

    if args.show_args:
        # TODO pete: Handle the case where different launch descriptions take different args?
        print_arguments_of_launch_description(
            launch_description=test_runs[0].get_launch_description()
        )
        sys.exit(0)

    _logger_.debug('Running integration test')
    try:
        results = runner.run()
        _logger_.debug('Done running integration test')

        if args.xmlpath:
            xml_report = unittestResultsToXml(
                test_results=results, name='{}.{}'.format(
                    args.package_name, test_file_basename
                )
            )
            xml_report.write(args.xmlpath, encoding='utf-8', xml_declaration=True)

        # There will be one result for every test run (see above where we load the tests)
        for result in results.values():
            if not result.wasSuccessful():
                sys.exit(1)

    except Exception as e:
        import traceback
        traceback.print_exc()
        parser.error(e)


if __name__ == '__main__':
    main()
