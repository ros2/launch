# Copyright 2019 Apex.AI, Inc.
# All rights reserved.

from contextlib import contextmanager


class SequentialTextChecker:
    """Helper class for asserting that text is found in a certain order."""

    def __init__(self, output):
        self._output = output
        self._array_index = 0  # Keeps track of how far we are into the output array
        self._substring_index = 0  # Keeps track of how far we are into an individual string

    def assertInText(self, msg):
        return self.assertInStdout(msg)

    def assertInStdout(self, msg):

        # Keep local copies of the array index and the substring index.  We only advance them
        # if we find a matching string.

        array_index = self._array_index
        substring_index = self._substring_index

        for text in self._output[array_index:]:
            found = text.find(msg, substring_index)

            if found != -1:
                # We found the string!  Update the search state for the next string
                substring_index = found + len(msg)
                self._array_index = array_index
                self._substring_index = substring_index
                return

            # We failed to find the string.  Go around the loop again
            array_index += 1
            substring_index = 0

        assert False, "{} not found in output".format(msg)


@contextmanager
def assertSequentialStdout(proc_output,
                           node):
    """
    Create a context manager used to check stdout occured in a specific order.

    :param proc_output:  The captured output from a test run
    :param node: The node that generated the output we intend to check
    """
    # TODO (pete baughman): Unify this node lookup [FTR2549]
    if isinstance(node, str):
        for proc in proc_output.processes():
            if node in proc.process_details['name']:
                node = proc
                break
        else:
            raise Exception("Did not find process matching name '{}'".format(node))

    # Get all the output from the node.  This will be a list of strings.  Each string may contain
    # multiple lines of output
    to_check = [p.text.decode('ascii') for p in proc_output[node]]
    checker = SequentialTextChecker(to_check)

    try:
        yield checker
    except Exception:
        # Do we need to log this, or give a better message?  Need to re-raise so we can
        # cause the test to fail
        raise
    finally:
        # We don't need to do anything to finalize the checker here
        pass
