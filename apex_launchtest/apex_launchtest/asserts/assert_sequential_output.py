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
                           proc):
    """
    Create a context manager used to check stdout occured in a specific order.

    :param proc_output:  The captured output from a test run
    :param proc: The process that generated the output we intend to check
    """
    # TODO (pete baughman): Unify this proc lookup [FTR2549]
    if isinstance(proc, str):
        for process_action in proc_output.processes():
            if proc in process_action.process_details['name']:
                proc = process_action
                break
        else:
            raise Exception("Did not find process matching name '{}'".format(proc))

    # Get all the output from the process.  This will be a list of strings.  Each string may
    # contain multiple lines of output
    to_check = [p.text.decode('ascii') for p in proc_output[proc]]
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
