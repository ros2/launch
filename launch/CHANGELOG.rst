^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package launch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.5 (2019-08-22)
------------------
* Add support for conditions in IncludeLaunchDescription actions (`#304 <https://github.com/ros2/launch/issues/304>`_)
* Contributors: Michel Hidalgo, chapulina

0.8.4 (2019-06-12)
------------------
* Fix build_cop `#214 <https://github.com/ros2/launch/issues/214>`_ (`#259 <https://github.com/ros2/launch/issues/259>`_)
* Fix get_launch_arguments to not crash on conditional sub entities (`#257 <https://github.com/ros2/launch/issues/257>`_)
* Use stderr logger instead of buffer (`#258 <https://github.com/ros2/launch/issues/258>`_)
* Line buffering of logger output (`#255 <https://github.com/ros2/launch/issues/255>`_)
* Contributors: Jacob Perron, Peter Baughman, ivanpauno

0.8.3 (2019-05-29)
------------------
* Changed IncludeLaunchDescription to not check declared arguments of subentities in order to work around an issue preventing nested arugments until a better fix can be done. (`#249 <https://github.com/ros2/launch/issues/249>`_)
* Fixed a bug where logging messages could be duplicated and improved logging's apperance on the CLI. (`#250 <https://github.com/ros2/launch/issues/250>`_)
* Contributors: Michel Hidalgo, ivanpauno

0.8.2 (2019-05-20)
------------------
* Moved some common code to LaunchDescriptionSource (`#234 <https://github.com/ros2/launch/issues/234>`_)
* Please flake8 on launch package. (`#241 <https://github.com/ros2/launch/issues/241>`_)
* Allow substitution in variable_name of LaunchConfiguration substitutions (`#235 <https://github.com/ros2/launch/issues/235>`_)
* Add support for custom launch log file handling (`#233 <https://github.com/ros2/launch/issues/233>`_)
* Contributors: Michel Hidalgo, ivanpauno

0.8.1 (2019-05-08)
------------------

0.8.0 (2019-04-13)
------------------
* Added SetEnvironmentVariable and UnsetEnvironmentVariable actions `#164 <https://github.com/ros2/launch/issues/164>`_ (`#216 <https://github.com/ros2/launch/issues/216>`_)
* Used one sentence per line in docs. (`#219 <https://github.com/ros2/launch/issues/219>`_)
* Added support for external ExecuteProcess output overrides. (`#218 <https://github.com/ros2/launch/issues/218>`_)
* Logged the launch logging config before running. (`#217 <https://github.com/ros2/launch/issues/217>`_)
* Fixed treating stderr output separate from stdout. (`#212 <https://github.com/ros2/launch/issues/212>`_)
* Replaced characters with marker when there is a decoding error. (`#202 <https://github.com/ros2/launch/issues/202>`_)
* Added LaunchLogger class. (`#145 <https://github.com/ros2/launch/issues/145>`_)
* Added test actions. (`#178 <https://github.com/ros2/launch/issues/178>`_)
* Fixed to close subprocess transport on execute action cleanup. (`#198 <https://github.com/ros2/launch/issues/198>`_)
* Updated logger.warn (deprecated) to logger.warning. (`#199 <https://github.com/ros2/launch/issues/199>`_)
* Dropped legacy launch package. (`#191 <https://github.com/ros2/launch/issues/191>`_)
* Migrated legacy launch API tests. (`#167 <https://github.com/ros2/launch/issues/167>`_)
* Updated to cancel Timers on shutdown. (`#181 <https://github.com/ros2/launch/issues/181>`_)
* Fixed timer global init of event handler. (`#184 <https://github.com/ros2/launch/issues/184>`_)
* Added support for required nodes (`#179 <https://github.com/ros2/launch/issues/179>`_)
* Updated to ensure event handlers add event to context locals. (`#177 <https://github.com/ros2/launch/issues/177>`_)
* Added OnProcessStart event handler. (`#171 <https://github.com/ros2/launch/issues/171>`_)
* Corrected OnProcessExit typing for Callable. (`#170 <https://github.com/ros2/launch/issues/170>`_)
* Removed whitespace in keyword arg. (`#169 <https://github.com/ros2/launch/issues/169>`_)
* Contributors: Dirk Thomas, Jacob Perron, Kyle Fazzari, Michel Hidalgo, Peter Baughman, Shane Loretz, William Woodall, ivanpauno, oswinso

0.7.3 (2018-12-13)
------------------
* Fixed deprecation warning related to collections.abc (`#158 <https://github.com/ros2/launch/pull/158>`_)
* Contributors: William Woodall

0.7.2 (2018-12-06)
------------------
* Changed the signit handler os it executes the shutdown event synchronously (`#156 <https://github.com/ros2/launch/issues/156>`_)
* Contributors: Jonathan Chapple, Steven! Ragnarök, William Woodall

0.7.1 (2018-11-16)
------------------
* Fixed setup.py versions (`#155 <https://github.com/ros2/launch/issues/155>`_)
* Contributors: Steven! Ragnarök

0.7.0 (2018-11-16)
------------------
* Fixed a bug to ensure that shutdown event is handled correctly (`#154 <https://github.com/ros2/launch/issues/154>`_)
  * There was a potential race condition in between when the shutdown event is emitted and the rest of the shutdown handling code.
  * This introduces an additional await to ensure that the event is emitted before proceeding.
* Fixed example to always use shell to avoid inconsistency of time being a shell command or executable (`#150 <https://github.com/ros2/launch/issues/150>`_)
* Added tests for class_tools module and fix is_a_subclass() (`#142 <https://github.com/ros2/launch/issues/142>`_)
* Added tests for the utilities module (`#143 <https://github.com/ros2/launch/issues/143>`_)
* Added 'handle_once' property for unregistering an EventHandler after one event (`#141 <https://github.com/ros2/launch/issues/141>`_)
* Added UnregisterEventHandler action (`#110 <https://github.com/ros2/launch/issues/110>`_)
* Changed LaunchService so that it returns ``1`` on caught exceptions from within launch (`#136 <https://github.com/ros2/launch/issues/136>`_)
* Added ability to define and pass launch arguments to launch files (`#123 <https://github.com/ros2/launch/issues/123>`_)
  * Added self descriptions for substitutions
  * Added tracebacks back to the output by default
  * Added new actions for declaring launch arguments
  * Added new method on LaunchDescription which gets all declared arguments within
  * Added ability to pass arguments when including a launch description
  * Added description for local variables used in Node action
  * Added ability to show and pass launch arguments on the command line
  * Added an accessor for the Condition of an Action
  * Signed-off-by: William Woodall <william@osrfoundation.org>
* Added UnsetLaunchConfiguration action and tests (`#134 <https://github.com/ros2/launch/issues/134>`_)
  * Signed-off-by: William Woodall <william@osrfoundation.org>
* Added GroupAction for conditionally including other actions and scoping (`#133 <https://github.com/ros2/launch/issues/133>`_)
  * Signed-off-by: William Woodall <william@osrfoundation.org>
* Added optional name argument to ExecuteProcess (`#129 <https://github.com/ros2/launch/issues/129>`_)
  * Signed-off-by: William Woodall <william@osrfoundation.org>
* Added a new pair of actions for pushing and popping launch configurations (`#128 <https://github.com/ros2/launch/issues/128>`_)
  * Signed-off-by: William Woodall <william@osrfoundation.org>
* Contributors: Dirk Thomas, Jacob Perron, Michael Carroll, William Woodall, dhood

0.6.0 (2018-08-20)
------------------
* Added a way to include other Python launch files (`#122 <https://github.com/ros2/launch/issues/122>`_)
  * Signed-off-by: William Woodall <william@osrfoundation.org>
* Implemented the concept of Action conditions (`#121 <https://github.com/ros2/launch/issues/121>`_)
  * Signed-off-by: William Woodall <william@osrfoundation.org>
* Added IncludeLaunchDescription action (`#120 <https://github.com/ros2/launch/issues/120>`_)
  * fixes `#115 <https://github.com/ros2/launch/issues/115>`_
  * Signed-off-by: William Woodall <william@osrfoundation.org>
* Contributors: William Woodall

0.5.2 (2018-07-17)
------------------
* Made a change to avoid reentrancy of signal handlers (`#99 <https://github.com/ros2/launch/issues/99>`_)
* Ignored warning for builtins A003 (`#100 <https://github.com/ros2/launch/issues/100>`_)
* Fixed exception when launch process with environment variables (`#96 <https://github.com/ros2/launch/issues/96>`_)
* Contributors: Shane Loretz, William Woodall, dhood

0.5.1 (2018-06-27)
------------------
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
