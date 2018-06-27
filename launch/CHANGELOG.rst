^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package launch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Changed the behavior when signaling SIGINT to subprocesses on Windows, where it now does SIGTERM instead, because SIGINT causes a ValueError about SIGINT being an unsupported signal number. (`#94 <https://github.com/ros2/launch/issues/94>`_)
* Fixed a bug by avoiding reentrancy in the SIGINT signal handler. (`#92 <https://github.com/ros2/launch/issues/92>`_)
* Various Windows fixes. (`#87 <https://github.com/ros2/launch/issues/87>`_)
  * LaunchService.run() now returns non-0 when there are exceptions in coroutines.
  * Updated ``launch_counters.py`` example for Windows.
  * Fixed a bug that would cause mismatched asyncio loops in some futures.
  * Addressed the fact that ``signal.SIGKILL`` doesn’t exist on Windows, so emulate it in our Event.
  * Fixed an issue that resulted in spurious asyncio errors in LaunchService test.
* Contributors: William Woodall, dhood

0.5.0 (2018-06-19)
------------------
* Fixed a bug where unclosed asyncio loops caused a traceback on the terminal on exit, but only in Python 3.5 (`#85 <https://github.com/ros2/launch/issues/85>`_)
* Changed to use variable typing in comments to support python 3.5 (`#81 <https://github.com/ros2/launch/issues/81>`_)
* New launch API (`#74 <https://github.com/ros2/launch/issues/74>`_)
  * See pull request for more details and links to architecture documentation and the design doc.
* Moved launch source files into launch.legacy namespace (`#73 <https://github.com/ros2/launch/issues/73>`_)
  * This was in preparation for the new launch API.
* [for launch.legacy] fixed a flake8 warning (`#72 <https://github.com/ros2/launch/issues/72>`_)
* [for launch.legacy] set zip_safe to avoid warning during installation (`#71 <https://github.com/ros2/launch/issues/71>`_)
* [for launch.legacy] Fix hang on keyboard interrupt (`#69 <https://github.com/ros2/launch/issues/69>`_)
  * When keyboard interrupt exception occurs loop.run_forever is called. But there is no loop.stop call. This causes a hang.
* Contributors: Devin, Dirk Thomas, William Woodall, dhood
