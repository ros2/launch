Signal Handling in ``launch``
=============================

This page describes how ``launch`` handles signals like ``SIGINT`` and ``SIGTERM``, how it shuts down the processes it started, and why it works the way it does.
It assumes you are already familiar with POSIX signals, and with the :class:`launch.LaunchService` and :class:`launch.actions.ExecuteProcess` classes.

Summary
-------

- While it is running, the launch service handles ``SIGINT`` and ``SIGTERM``, and either one starts a normal shutdown by emitting a :class:`launch.events.Shutdown` event.
- In response to that event, each running process is sent ``SIGINT``, then ``SIGTERM`` after ``sigterm_timeout`` seconds, and then ``SIGKILL`` after another ``sigkill_timeout`` seconds, stopping as soon as the process exits.
- The initial ``SIGINT`` is skipped when ``launch`` thinks the process already got ``SIGINT`` from the terminal, i.e. when the shutdown was caused by ``SIGINT`` and ``launch`` is running interactively.

Signals in ROS 2
----------------

There are two common ways that a ROS 2 system is asked to stop, and they deliver signals differently:

- pressing ctrl-c in a terminal causes the terminal to send ``SIGINT`` to every process in the foreground process group, not just to the process you started
- service managers, container runtimes, and tools like ``kill`` usually send ``SIGTERM`` to a single process, typically the top level one

Programs also react to these signals differently:

- ROS 2 nodes that use ``rclcpp`` or ``rclpy`` install handlers for both ``SIGINT`` and ``SIGTERM`` by default, and treat them the same, by shutting down the ROS context
- plain Python programs turn ``SIGINT`` into a :exc:`KeyboardInterrupt`, but leave ``SIGTERM`` with its default action, which terminates the process without running ``finally`` blocks or ``atexit`` handlers
- many other programs only clean up gracefully on ``SIGINT``, if they handle signals at all

For comparison, ``ros2 run`` starts a single process and forwards any ``SIGINT`` or ``SIGTERM`` it receives to that process.
``launch`` has to manage many processes and keep its event loop running while they shut down, so rather than forwarding signals directly, it turns them into a shutdown of the launch service, which in turn shuts down the processes.

How ``launch`` Receives Signals
-------------------------------

:meth:`launch.LaunchService.run` and :meth:`launch.LaunchService.run_async` install handlers for ``SIGINT`` and ``SIGTERM`` while they are running, and restore the previous handlers when they return.
Python only allows signal handlers to be set from the main thread, which is one of the reasons the launch service has to be run from the main thread.

The handlers are installed using :class:`launch.utilities.AsyncSafeSignalManager`, rather than with :func:`signal.signal` directly or with ``asyncio``'s ``loop.add_signal_handler()``.
A regular Python signal handler can run between any two bytecode instructions in the main thread, so it cannot safely interact with the ``asyncio`` event loop, and ``loop.add_signal_handler()``, which does solve that problem, is not available on Windows.

Instead, ``AsyncSafeSignalManager`` uses :func:`signal.set_wakeup_fd`:

- when entering its context, it creates a socket pair, passes the write end to :func:`signal.set_wakeup_fd`, and registers the read end with the event loop using ``loop.add_reader()``
- when a signal is received, Python's C signal handler writes the signal number to the socket, which wakes up the event loop, which then calls the handler registered for that signal from the event loop's thread
- on Windows, where the default ``ProactorEventLoop`` does not support ``add_reader()``, the read end is watched by a ``SelectorEventLoop`` in a background thread instead, which schedules the handler on the main event loop with ``call_soon_threadsafe()``
- if a wakeup file descriptor was already set, e.g. by an outer ``AsyncSafeSignalManager`` or by ``asyncio`` itself, the signal number is forwarded to it as well, so that managers can be nested

There is a catch with :func:`signal.set_wakeup_fd`, which is that Python only writes to the wakeup file descriptor for signals that have a Python handler set with :func:`signal.signal`.
Python sets ``signal.default_int_handler`` as the handler for ``SIGINT`` at startup, so ``SIGINT`` always gets written to the wakeup file descriptor, but ``SIGTERM`` is left with its default action, so without some other handler ``SIGTERM`` would just terminate the ``launch`` process immediately.
To address this, while its context is active, ``AsyncSafeSignalManager`` sets a Python handler for each signal it manages:

- if the signal already has a Python handler, e.g. ``signal.default_int_handler`` for ``SIGINT``, then the new handler calls it, so that its behavior is preserved
- otherwise, e.g. ``SIG_DFL`` for ``SIGTERM``, the new handler does nothing, and exists only so that the wakeup file descriptor gets written to

The previous handlers are restored when the context exits, or when a signal's handler is removed with ``handle(signum, None)``.
This also means that a signal which was being ignored, e.g. ``SIGINT`` for a background job started by a non-interactive shell, is not ignored while the launch service is running.

Because of this, ``SIGINT`` still raises :exc:`KeyboardInterrupt` in the main thread, so the launch service catches and ignores :exc:`KeyboardInterrupt` in its run loops, since the signal is handled through the wakeup file descriptor instead.

What Happens After a Signal Is Received
---------------------------------------

The first time ``SIGINT`` or ``SIGTERM`` is received, the launch service logs ``caught SIGINT`` (or ``caught SIGTERM``) and emits a :class:`launch.events.Shutdown` event, with ``due_to_sigint`` set to ``True`` only if the signal was ``SIGINT``.

After that, more signals are logged but have no other effect.
This is true for the same signal again, e.g. pressing ctrl-c twice, and for the other signal, e.g. ``SIGTERM`` after ``SIGINT``, because the launch service only emits one shutdown event, and each process only starts its shutdown sequence once.
So once shutdown has started, it continues until the processes exit or the escalation timers expire.
``SIGQUIT`` (``ctrl-\``) is not handled by ``launch``, so it has its default action and terminates ``launch`` immediately, and when it comes from a terminal it is also sent to the processes, which will likely terminate them too.

Like any other event, the shutdown event is passed to any matching event handlers, e.g. :class:`launch.event_handlers.OnShutdown`.
Each :class:`launch.actions.ExecuteLocal` action (the base class of :class:`launch.actions.ExecuteProcess` and ``launch_ros.actions.Node``) also handles it by starting to shut down its process, if that process is still running:

- a timer is started to send ``SIGTERM`` after ``sigterm_timeout`` seconds (5 by default)
- a timer is started to send ``SIGKILL`` after ``sigterm_timeout + sigkill_timeout`` seconds (10 by default)
- ``SIGINT`` is sent to the process, unless the shutdown was caused by ``SIGINT`` and ``launch`` is running interactively

``sigterm_timeout`` and ``sigkill_timeout`` can be set on each action, or for all actions with the launch configurations of the same names.

The rule for whether or not to send ``SIGINT`` is ``send_sigint = not due_to_sigint or context.noninteractive``, which works out to:

.. list-table::
   :header-rows: 1

   * - Cause of shutdown
     - Interactive
     - Non-interactive
   * - ``SIGINT``
     - not sent
     - sent
   * - ``SIGTERM``, or anything else
     - sent
     - sent

Separately, a single process can be shut down with the :class:`launch.events.process.ShutdownProcess` event, which always sends ``SIGINT`` first.

Why ``SIGINT`` Is Not Sent Again After ctrl-c
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``launch`` does not put the processes it starts into a new process group, so when ``launch`` is run from a terminal, they are in the same foreground process group as ``launch``.
That means when the user presses ctrl-c, the terminal sends ``SIGINT`` to ``launch`` and to all of its processes at the same time.
This is by design, and it is the same as what happens with ROS 1's ``roslaunch``, or with the child processes of a shell script, or of a Python script that uses :mod:`subprocess`.

Since the processes already got ``SIGINT``, ``launch`` does not send them another one, and only starts the escalation timers.
Sending a second ``SIGINT`` could interrupt a process that is already cleaning up, e.g. a Python program would get a second :exc:`KeyboardInterrupt` while running its ``finally`` blocks.

Why ``SIGINT`` Is Sent First After ``SIGTERM``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Unlike ctrl-c, ``SIGTERM`` is usually sent only to ``launch``, so the processes have not received anything yet.
The same is true for shutdowns that are not caused by a signal at all, e.g. when the launch service becomes idle, when :meth:`launch.LaunchService.shutdown` is called, or when a :class:`launch.actions.Shutdown` action is executed.

In these cases ``launch`` sends the processes ``SIGINT``, not ``SIGTERM``, so that the shutdown sequence is the same no matter how the shutdown was started.
This also gives programs that only clean up on ``SIGINT`` a chance to do so, and ROS 2 nodes treat ``SIGINT`` and ``SIGTERM`` the same anyway.

Interactive vs. Non-interactive
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The assumption that the processes got ``SIGINT`` from the terminal only holds when ``launch`` is actually running in a terminal and the ``SIGINT`` came from ctrl-c.
When ``launch`` is started by a script, a test, or some other program, and that program sends ``SIGINT`` to only the ``launch`` process, then nothing else is going to send ``SIGINT`` to the processes.

The ``noninteractive`` option exists for these cases (see `ros2/launch#475 <https://github.com/ros2/launch/pull/475>`_).
When it is ``True``, ``launch`` always sends ``SIGINT`` to its processes during shutdown.
It defaults to ``False`` in :class:`launch.LaunchService`, while ``ros2 launch`` sets it to ``True`` if stdin is not a tty, or if ``--noninteractive`` (``-n``) is passed.

Note that if ``launch`` is running interactively and something sends ``SIGINT`` to only the ``launch`` process, e.g. with ``kill -INT <pid>``, then the processes will not get ``SIGINT``, and will not start to shut down until the ``SIGTERM`` timer expires.
To avoid this, send ``SIGINT`` to the whole process group, send ``SIGTERM`` to ``launch`` instead, or use ``--noninteractive``.

Platform and Deployment Notes
-----------------------------

Windows
^^^^^^^

On Windows, ``SIGINT`` cannot be sent to a single process, so :class:`launch.actions.ExecuteLocal` logs a warning and sends ``SIGTERM`` instead, which on Windows terminates the process immediately.
So on Windows, processes generally only get a chance to shut down gracefully after an interactive ctrl-c in the console.
Also, Python does not define ``signal.SIGKILL`` on Windows, so the ``SIGKILL`` step uses the subprocess transport's ``kill()`` method instead.

Containers
^^^^^^^^^^

Container runtimes stop a container by sending ``SIGTERM`` to its main process, followed by ``SIGKILL`` after a grace period, which by default is 10 seconds for ``docker stop`` and 30 seconds for Kubernetes (``terminationGracePeriodSeconds``).
With the default timeouts, ``launch`` can take up to 10 seconds to shut down its processes, so with Docker's default the container may be killed before ``launch`` is done.
Either increase the grace period, e.g. ``docker stop --time 15``, or reduce ``sigterm_timeout`` and ``sigkill_timeout``.

If ``launch`` is the main process of the container then it is PID 1 in the container's PID namespace, and the kernel does not deliver signals to PID 1 unless it has installed a handler for them.
Since ``launch`` installs handlers for ``SIGINT`` and ``SIGTERM`` it still gets the ``SIGTERM`` from ``docker stop``, but using an init process, e.g. ``docker run --init``, is still a good idea so that orphaned processes get reaped.

systemd
^^^^^^^

By default, ``systemd`` stops a service by sending ``SIGTERM`` to every process in the service's cgroup at the same time (``KillMode=control-group``).
That means the processes started by ``launch`` will get ``SIGTERM`` directly, in addition to the ``SIGINT`` from ``launch``, and processes that do not handle ``SIGTERM`` will be terminated immediately.
Setting ``KillMode=mixed`` makes ``systemd`` send ``SIGTERM`` only to the main process, which lets ``launch`` shut down its processes, and ``TimeoutStopSec=`` should be longer than ``sigterm_timeout + sigkill_timeout`` so that ``systemd`` does not send ``SIGKILL`` to everything first.

Wrapper Scripts and ``shell=True``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``launch`` only sends signals to the process it started, not to that process's children.
So if that process is a shell, e.g. because of ``shell=True``, a ``prefix``, or a wrapper script, then the signals may not reach the program you actually care about, depending on whether or not the shell forwards them.
This does not affect the ``SIGINT`` from an interactive ctrl-c, since that goes to the whole process group.
Wrapper scripts should use ``exec`` to replace the shell with the program, e.g. ``exec "$@"``.

History
-------

The original version of ``launch`` for ROS 2 (`ros2/launch#74 <https://github.com/ros2/launch/pull/74>`_) installed its handlers directly with :func:`signal.signal`.
``SIGINT`` started a graceful shutdown as described above, but ``SIGTERM`` and ``SIGQUIT`` canceled the launch service's run task immediately, and logged a warning that doing so could leave orphaned processes.

Later, signal handling was moved into the ``asyncio`` event loop with ``AsyncSafeSignalManager`` (`ros2/launch#476 <https://github.com/ros2/launch/pull/476>`_), which at the time relied only on :func:`signal.set_wakeup_fd`.
As described above, that does not work for signals without a Python handler, so from then on ``SIGTERM`` and ``SIGQUIT`` terminated ``launch`` right away, without shutting down its processes (see `ros2/launch#666 <https://github.com/ros2/launch/issues/666>`_).

`ros2/launch#712 <https://github.com/ros2/launch/pull/712>`_ fixed this by having ``AsyncSafeSignalManager`` set Python handlers while it is active, and changed ``SIGTERM`` to start the same graceful shutdown as ``SIGINT``, rather than canceling the run task.
It also stopped handling ``SIGQUIT``, leaving it with its default action.
