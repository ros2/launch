^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package launch_testing
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.5 (2019-11-13)
------------------
* Make launch_testing.markers.retry_on_failure decorator more robust. (`#352 <https://github.com/ros2/launch/issues/352>`_)
* Contributors: Michel Hidalgo

0.9.4 (2019-11-08)
------------------
* Fix a small typo in the launch_testing README. (`#351 <https://github.com/ros2/launch/issues/351>`_)
* Contributors: Chris Lalancette

0.9.3 (2019-10-23)
------------------

0.9.2 (2019-10-23)
------------------
* Support launch test reruns when using pytest (`#348 <https://github.com/ros2/launch/issues/348>`_)
* Support CLI commands testing (`#279 <https://github.com/ros2/launch/issues/279>`_)
* Contributors: Michel Hidalgo

0.9.1 (2019-09-25)
------------------
* Optionally remove ready fn arg from generate_test_description (`#322 <https://github.com/ros2/launch/issues/322>`_)
* Contributors: Michel Hidalgo, Peter Baughman

0.9.0 (2019-09-18)
------------------
* install package manifest (`#330 <https://github.com/ros2/launch/issues/330>`_)
* Unindent setup.cfg options. (`#326 <https://github.com/ros2/launch/issues/326>`_)
* Use renamed remove_ansi_escape_sequences. (`#302 <https://github.com/ros2/launch/issues/302>`_)0
* Enable launch test discovery in pytest (`#312 <https://github.com/ros2/launch/issues/312>`_)
* Support LaunchService injection into pre-shutdown tests. (`#308 <https://github.com/ros2/launch/issues/308>`_)
* Add assertWaitForStartup method to match assertWaitForShutdown (`#278 <https://github.com/ros2/launch/issues/278>`_)
* Fix a simple typo in an error message. (`#301 <https://github.com/ros2/launch/issues/301>`_)
* Fix launch_testing output filtering (`#296 <https://github.com/ros2/launch/issues/296>`_)
* Revert "Revert "[execute_process] emulate_tty configurable and defaults to true"" (`#277 <https://github.com/ros2/launch/issues/277>`_)
* Fix formatting (`#262 <https://github.com/ros2/launch/issues/262>`_)
* Fix proc lookup for processes with multiple command-line arguments (`#229 <https://github.com/ros2/launch/issues/229>`_)
* Remove ros domain ID dependency (`#256 <https://github.com/ros2/launch/issues/256>`_)
* Contributors: Chris Lalancette, Dirk Thomas, Esteve Fernandez, Michel Hidalgo, Peter Baughman, William Woodall, ivanpauno

0.8.3 (2019-05-29)
------------------
* Changed behavior to use ``--isolated`` if no ``ROS_DOMAIN_ID`` is set to help parallel testing. (`#251 <https://github.com/ros2/launch/issues/251>`_)
* Contributors: Peter Baughman

0.8.2 (2019-05-20)
------------------
* add non-asserting waitFor method (`#243 <https://github.com/ros2/launch/issues/243>`_)
* Enable reuse of launch testing functionality (`#236 <https://github.com/ros2/launch/issues/236>`_)
* Stop randomizing ROS_DOMAIN_ID by default in launch tests (`#240 <https://github.com/ros2/launch/issues/240>`_)
* Contributors: Dirk Thomas, Michel Hidalgo

0.8.1 (2019-05-08)
------------------

0.8.0 (2019-04-13)
------------------
* Added test actions. (`#178 <https://github.com/ros2/launch/issues/178>`_)
* Fixed test_env_testing test (`#200 <https://github.com/ros2/launch/issues/200>`_)
* Dropped legacy launch package. (`#191 <https://github.com/ros2/launch/issues/191>`_)
* Migrated legacy launch API tests. (`#167 <https://github.com/ros2/launch/issues/167>`_)
* Contributors: Dirk Thomas, Michel Hidalgo, ivanpauno

0.7.3 (2018-12-13)
------------------

0.7.2 (2018-12-06)
------------------

0.7.1 (2018-11-16)
------------------
* Fixed setup.py versions (`#155 <https://github.com/ros2/launch/issues/155>`_)
* Contributors: Steven! Ragnarök

0.7.0 (2018-11-16)
------------------
* Fixed lint warnings from invalid escape sequences (`#151 <https://github.com/ros2/launch/issues/151>`_)
  Use raw strings for regex patterns to avoid warnings.
* Fixed linter errors from `#131 <https://github.com/ros2/launch/issues/131>`_. (`#132 <https://github.com/ros2/launch/issues/132>`_)
* Added class to provide some limitted testing options (`#131 <https://github.com/ros2/launch/issues/131>`_)
* Moved ``launch_testing`` into ``launch_testing.legacy`` namespace (`#130 <https://github.com/ros2/launch/issues/130>`_)
* Contributors: Dirk Thomas, Jacob Perron, Steven! Ragnarök

0.6.0 (2018-08-20)
------------------

0.5.2 (2018-07-17)
------------------

0.5.1 (2018-06-27)
------------------

0.5.0 (2018-06-19)
------------------
* Updated to use new launch.legacy namespace (`#73 <https://github.com/ros2/launch/issues/73>`_)
* Contributors: Dirk Thomas, Mikael Arguedas, William Woodall
