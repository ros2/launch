# Examples

## `good_proc.test.py`

Usage:
> apex_launchtest examples/good_proc.test.py

This test checks a process called good_proc (source found in the [example_processes folder](../example_processes)).
good_proc is a simple python process that prints "Loop 1, Loop2, etc. every second until it's terminated with ctrl+c.
The test will launch the process, wait for a few loops to complete by monitoring stdout, then terminate the process
and run some post-shutdown checks.

The pre-shutdown tests check that "Loop 1, Loop 2, Loop 3, Loop 4"
are all printed to stdout.  Once this test finishes, the process under test is shut down

After shutdown, we run a similar test that checks more output, and also checks the
order of the output.  `test_out_of_order` demonstrates that the `assertSequentialStdout`
context manager is able to detect out of order stdout.

## `args.test.py`

Usage to view the arguments:
>apex_launchtest examples/args.test.py --show-args

Usage to run the test:
>apex_launchtest examples/args.test.py dut_arg:=hey

This example shows how to pass arguments into an apex_launchtest.  The arguments are made avilable
in the launch description via a launch.substitutions.LaunchConfiguration.  The arguments are made
available to the test cases via a self.test_args dictionary

This example will fail if no arguments are passed.

## `example_test_context.test.py`

Usage:
> apex_launchtest examples/example_test_context.test.py

This example shows how the `generate_test_description` function can return a tuple where the second
item is a dictionary of objects that will be injected into the individual test cases.  Tests that
wish to use elements of the test context can add arguments with names matching the keys of the dictionary.
