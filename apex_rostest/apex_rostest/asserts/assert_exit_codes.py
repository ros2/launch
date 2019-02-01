# Copyright 2019 Apex.AI, Inc.
# All rights reserved.

EXIT_OK = 0
EXIT_SIGINT = 130
EXIT_SIGQUIT = 131
EXIT_SIGKILL = 137
EXIT_SIGSEGV = 139


def assertExitCodes(proc_info, allowable_exit_codes=[EXIT_OK]):
    """Check the exit codes of the nodes under test.

    :param iterable proc_info: A list of proc_info objects provided by the test framework to be
    checked
    """
    # Sanity check that the user didn't pass in something crazy for allowable exit codes
    for code in allowable_exit_codes:
        assert isinstance(code, int), "Provided exit code {} is not an int".format(code)

    for info in proc_info:
        assert info.returncode in allowable_exit_codes, "Proc {} exited with code {}".format(
            info.process_name,
            info.returncode
        )
