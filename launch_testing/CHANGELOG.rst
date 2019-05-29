^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package launch_testing
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
