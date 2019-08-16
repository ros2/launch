# Examples

## `good_proc_launch_test.py`

Usage:
```sh
launch_test good_proc_launch_test.py
```
This test checks a process called good_proc (source found in the [example_processes folder](../../../example_processes)).
good_proc is a simple python process that prints "Loop 1, Loop2, etc. every second until it's terminated with ctrl+c.
The test will launch the process, wait for a few loops to complete by monitoring stdout, then terminate the process
and run some post-shutdown checks.

The pre-shutdown tests check that "Loop 1, Loop 2, Loop 3, Loop 4"
are all printed to stdout.  Once this test finishes, the process under test is shut down

After shutdown, we run a similar test that checks more output, and also checks the
order of the output.  `test_out_of_order` demonstrates that the `assertSequentialStdout`
context manager is able to detect out of order stdout.


## `terminating_proc_launch_test.py`

Usage:
```sh
launch_test terminating_proc_launch_test.py
```

This test checks proper functionality of the _terminating\_proc_ example (source found in the [example_processes folder](../../../example_processes)).

## `args_launch_test.py`

Usage to view the arguments:
```sh
launch_test args_launch_test.py --show-args
```
Usage to run the test:
```sh
launch_test args_launch_test.py dut_arg:=hey
```
This example shows how to pass arguments into a launch test.  The arguments are made avilable
in the launch description via a launch.substitutions.LaunchConfiguration.  The arguments are made
available to the test cases via a self.test_args dictionary

This example will fail if no arguments are passed.

## `context_launch_test.py`

Usage:
```sh
launch_test context_launch_test.py
```
This example shows how the `generate_test_description` function can return a tuple where the second
item is a dictionary of objects that will be injected into the individual test cases.  Tests that
wish to use elements of the test context can add arguments with names matching the keys of the dictionary.
