# Copyright 2019 Apex.AI, Inc.
# All rights reserved.

import launch.actions

NO_CMD_ARGS = object()


def _proc_to_name_and_args(proc):
    # proc is a launch.actions.ExecuteProcess
    return "{} {}".format(
        proc.process_details['name'],
        " ".join(proc.process_details['cmd'][1:])
    )


def _assertInStdoutByProcessAction(
        proc_output,
        msg,
        process_action):

    for output in proc_output[process_action]:
        if msg in output.text.decode('ascii'):
            return
    else:
        assert False, "Did not find '{}' in output for {}".format(
            msg,
            _proc_to_name_and_args(process_action)
        )


def _assertInStdoutByStringProcessName(
        proc_output,
        msg,
        node_name,
        cmd_args,
        strict_node_matching):

    # Ensure that the combination node_name and cmd_args are not ambiguous.  If they are,
    # we need to cause an error to bubble up in order to alert the test writer that we may not
    # be checking what they intend to check

    def name_match_fn(proc):
        return node_name in proc.process_details['name']

    def cmd_match_fn(proc):
        if cmd_args is None:
            return True
        elif cmd_args is NO_CMD_ARGS:
            return len(proc.process_details['cmd']) == 1
        else:
            return cmd_args in proc.process_details['cmd'][1:]

    unique_procs = proc_output.processes()
    matching_procs = [proc for proc in unique_procs if name_match_fn(proc) and cmd_match_fn(proc)]

    if len(matching_procs) == 0:
        node_names = ', '.join(sorted([_proc_to_name_and_args(proc) for proc in unique_procs]))

        raise Exception(
            "Did not find any processes matching name '{}' and args '{}'. Procs: {}".format(
                node_name,
                cmd_args,
                node_names
            )
        )
    elif strict_node_matching and len(matching_procs) > 1:
        node_names = ', '.join(sorted([_proc_to_name_and_args(proc) for proc in matching_procs]))
        raise Exception(
            "Found multiple processes matching name '{}' and cmd_args '{}'. Procs: {}".format(
                node_name,
                cmd_args,
                node_names
            )
        )

    for proc in matching_procs:  # Nominally just one matching proc
        for output in proc_output[proc]:
            if msg in output.text.decode('ascii'):
                return
    else:
        assert False, "Did not find '{}' in output for process {} {}".format(
            msg,
            node_name,
            cmd_args
        )


def assertInStdout(proc_output,
                   msg,
                   node,
                   cmd_args=None,
                   *,
                   strict_node_matching=True):
    """Assert that 'msg' was found in the standard out of a node.

    :param proc_output: The process output captured by apex_rostest.  This is usually injected
    into test cases as self._proc_output
    :type proc_output: An apex_rostest.IoHandler

    :param msg: The message to search for
    :type msg: string

    :param node: The node whose output will be searched
    :type node: A string (search by process name) or a launch.actions.ExecuteProcess object

    :param cmd_args: Optional.  If 'node' is a string, cmd_args will be used to disambiguate
    processes with the same name.  Pass apex_rostest.asserts.NO_CMD_ARGS to match a node without
    command arguments
    :type cmd_args: string

    :param strict_node_matching: Optional (default True), If node is a string and the combination
    of node and cmd_args matches multiple processes, then strict_node_matching=True will raise
    an error.
    :type strict_node_matching: bool
    """
    # Depending on the type of 'node' we're going to dispatch this a little differently
    if isinstance(node, launch.actions.ExecuteProcess):
        _assertInStdoutByProcessAction(
            proc_output,
            msg,
            node
        )
    elif isinstance(node, str):
        _assertInStdoutByStringProcessName(
            proc_output,
            msg,
            node,
            cmd_args,
            strict_node_matching
        )
    else:
        raise TypeError(
            "node argument must be 'ExecuteProcess' or 'str' not {}".format(type(node))
        )
