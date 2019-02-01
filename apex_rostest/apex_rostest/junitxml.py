# Copyright 2019 Apex.AI, Inc.
# All rights reserved.


import xml.etree.ElementTree as ET


def unittestResultsToXml(*, name="apex_rostest", test_results={}):
    """
    Serialize multiple unittest.TestResult objects into an XML document.

    A testSuites element will be the root element of the document.
    """
    # The test_suites element is the top level of the XML result.
    # apex_rostest results contain two test suites - one from tests that ran while nodes were
    # active, and one from tests that ran after nodes were shut down
    test_suites = ET.Element('testsuites')
    test_suites.set('name', name)
    # test_suites.set('time', ????) skipping 'time' attribute

    # To get tests, failures, and errors, we just want to iterate the results once
    tests = 0
    failures = 0
    errors = 0

    for result in test_results.values():
        tests += result.testsRun
        failures += len(result.failures)
        errors += len(result.errors)

    test_suites.set('tests', str(tests))
    test_suites.set('failures', str(failures))
    test_suites.set('errors', str(errors))

    for (key, value) in test_results.items():
        test_suites.append(unittestResultToXml(key, value))

    return ET.ElementTree(test_suites)


def unittestResultToXml(name, test_result):
    """Serialize a single unittest.TestResult to an XML element."""
    test_suite = ET.Element('testsuite')
    test_suite.set('name', name)
    test_suite.set('tests', str(test_result.testsRun))
    test_suite.set('failures', str(len(test_result.failures)))
    test_suite.set('errors', str(len(test_result.errors)))

    # TODO(pete.baughman): Consider adding time, skipped, package, and/or file attributes as well
    # as 'testcase' sub-elements

    return test_suite
