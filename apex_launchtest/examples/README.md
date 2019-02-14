# Examples

## `good_proc.test.py`

Usage:
> apex_launchtest examples/good_proc.test.py

This test launches a python process that spins forever waiting for ctrl+c
to shut it down.

The pre-shutdown tests check that "Loop 1, Loop 2, Loop 3, Loop 4"
are all printed to stdout.

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
