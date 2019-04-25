# launch_testing

This tool is a framework for ROS2 integration testing using the [ros2 style launch description](https://github.com/ros2/launch/blob/master/ros2launch/examples/example.launch.py).
It works similarly to rostest, but makes it easier to inspect the processes under test. For example:

  * The exit codes of all processes are available to the tests.
    Tests can check that all processes shut down normally, or with specific exit codes.
    Tests can fail when a process dies unexpectedly.
  * The stdout and stderr of all processes are available to the tests.
  * The command-line used to launch the processes are avilalbe to the tests.
  * Some tests run concurrently with the launch and can interact with the running processes.

## Quick start example

Start with the `launch_testing` example [`good_proc.test.py`](examples/good_proc.test.py).

Run the example by doing:

```sh
launch_test launch_testing/examples/good_proc.test.py
```

`launch_test` will launch the nodes found in the `generate_test_description` function, run the tests from the `TestGoodProcess` class, shut down the launched nodes, and then run the tests from the `TestNodeOutput` class.

#### The Launch Description


```python
def generate_test_description(ready_fn):

    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=[path_to_process],
        ),

        # Start tests right away - no need to wait for anything in this example
        launch.actions.OpaqueFunction(function=lambda context: ready_fn()),
    ])
```

The `generate_test_description` function should return a `launch.LaunchDescription` object that launches the system to be tested.
It should also call the `ready_fn` that is passed in to signal when the tests should start.  In the `good_proc.test.py` example, there is no need to delay the start of the tests so the `ready_fn` is called concurrently when the launching of the process under test.

#### Active Tests

Any classes that inherit from `unittest.TestCase` and not decorated with the `post_shutdown_test` descriptor will be run concurrently with the proccess under test.
These tests are expected to interact with the running processes in some way.

#### Post-Shutdown Tests

Any classes that inherit `from unittest.TestCase` that are decorated with the `post_shutdown_test` descriptor will be run after the launched processes have been shut down.
These tests have access to the exit codes and the stdout of all of the launched processes, as well as any data created as a side-effect of running the processes.

#### Exit Codes and Standard Out

The `launch_testing` framework automatically adds some member fields to each test case so that the tests can access process output and exit codes.

 - `self.proc_info` - a [ProcInfoHandler object](launch_testing/proc_info_handler.py)
 - `self.proc_output` - an [IoHandler object](launch_testing/io_handler.py)

These objects provide dictionary like access to information about the running processes.
They also contain methods that the active tests can use to wait for a process to exit or to wait for specific output.

## Assertions

The `launch_testing` framework automatically records all stdout from the launched processes as well as the exit codes from any processes that are launched.
This information is made available to the tests via the `proc_info` and `proc_output` object.
These objects can be used by one of several assert methods to check the output or exit codes of the process:

`launch_testing.asserts.assertInStdout(proc_output, msg, proc, cmd_args=None, *, strict_proc_matching=True)`

Asserts that a message is found in the stdout of a particular process.

  - `msg`:

    The text to look for in the process standard out

  -` proc`:

    Either the process name as a string, or a `launch.actions.ExecuteProcess` object that was used to start the process.
    Pass `None` or an empty string to search all processes.

  - `cmd_args`:

    When looking up processes by process by name, `cmd_args` can be used to disambiguate multiple processes with the same name.

  - `strict_proc_matching`:

    When looking up a process by name, `strict_proc_matching=True` will make it an error to match multiple processes.
    This prevents an assert from accidentally passing if the output came from a different process than the one the user was expecting.

`launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[EXIT_OK], proc, cmd_args=None, *, strict_proc_matching=True)`

Asserts that the specified processes exited with a particular exit code.

  - `allowable_exit_codes`:

    A list of allowable exit codes.
    By default `EXIT_OK` (0) plus `EXIT_FORCED` (1) on Windows.
    Other exit codes provided are `EXIT_SIGINT` (130), `EXIT_SIGQUIT` (131), `EXIT_SIGKILL` (137) and `EXIT_SIGSEGV` (139).

  - The `proc`, `cmd_args`, and `strict_proc_matching` arguments behave the same way as in `assertInStdout`.
    By default, assert on the exit codes of all processes.

`launch_testing.asserts.assertSequentialStdout(proc_output, proc, cmd_args=None)`

Asserts that standard out was seen in a particular order.

  - `proc` and `cmd_args`:

    These arguments are the same as in `assertInStdout` and `assertExitCodes`, however it is not possible to match multiple processes because there is no way to determine the order of stdout that came from multiple processes.

Returns a context manager that will check that a series of assertions happen in order.

As an example, consider:

```python
with assertSequentialStdout(self.proc_output, "proc_name") as cm:
    cm.assertInStdout("Loop 1")
    cm.assertInStdout("Loop 2")
    cm.assertInStdout("Loop 3")
```

#### Waiting for Output or Exit Codes

The `ActiveTests` can also call methods that wait for particular output or a particular process to exit or time out.
These asserts are methods on the `proc_output` and `proc_info` objects.

`proc_output.assertWaitFor(msg, proc=None, cmd_args=None, *, strict_proc_matching=True, timeout=10)`

  - `msg`, `proc`, `cmd_args`, and `strict_proc_matching`:

    These arguments work the same as in other assert methods.
    By default, this method waits on output from any process.

  - `timeout`:

    The amount of time to wait before raising an `AssertionError`.

`proc_info.assertWaitForShutdown(proc, cmd_args=None, *, timeout=10)`

  - `proc` and `cmd_args`:

    These arguments work the same as in other assertions, but it is not possible to wait on multiple processes to shut down.

  - `timeout`:

    The amount of time to wait before raising an `AssertionError`

## Arguments

`launch_test` uses the same [syntax as ros2 launch](https://github.com/ros2/launch/pull/123) to pass arguments to tests.

Arguments are declared in the launch description and can be accessed by the test via a `test_args` dictionary that's injected into the tests similar to `proc_info` and `proc_output`.

```sh
launch_test --show-args examples/args.test.py
launch_test examples/args.test.py dut_arg:=value
```

See the [launch_testing example with arguments](examples/args.test.py) for further reference.

## Using CMake

To run launch tests from a CMakeLists.txt file, you'll need to declare a dependency on
`launch_testing_ament_cmake` in your `package.xml`.

Then, in the CMakeLists.txt file, add:

```cmake
find_package(launch_testing_ament_cmake)
add_launch_test(test/name_of_test.test.py)
```

Arguments can be passed to the tests via the CMake function, too:

```cmake
add_launch_test(
  test/test_with_args.test.py
  ARGS "arg1:=foo"
)
```
