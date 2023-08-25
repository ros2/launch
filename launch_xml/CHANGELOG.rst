^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package launch_xml
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.0 (2023-08-21)
------------------

2.2.1 (2023-07-11)
------------------
* Improve launch file parsing error messages (`#626 <https://github.com/ros2/launch/issues/626>`_)
* Contributors: Timon Engelke

2.2.0 (2023-06-07)
------------------

2.1.0 (2023-04-27)
------------------

2.0.1 (2023-04-12)
------------------

2.0.0 (2023-04-11)
------------------

1.4.1 (2023-02-24)
------------------
* Fixed typos (`#692 <https://github.com/ros2/launch/issues/692>`_)
* Contributors: Alejandro Hernández Cordero

1.4.0 (2023-02-14)
------------------
* Expose emulate_tty to xml and yaml launch (`#669 <https://github.com/ros2/launch/issues/669>`_)
* Expose sigterm_timeout and sigkill_timeout to xml frontend (`#667 <https://github.com/ros2/launch/issues/667>`_)
* [rolling] Update maintainers - 2022-11-07 (`#671 <https://github.com/ros2/launch/issues/671>`_)
* Contributors: Aditya Pande, Audrow Nash

1.3.0 (2022-11-02)
------------------

1.2.0 (2022-09-13)
------------------

1.1.0 (2022-04-29)
------------------

1.0.1 (2022-04-13)
------------------

1.0.0 (2022-04-12)
------------------

0.23.1 (2022-04-08)
-------------------
* Fix sphinx directive to cross-ref Launch method (`#605 <https://github.com/ros2/launch/issues/605>`_)
* Contributors: Abrar Rahman Protyasha

0.23.0 (2022-03-30)
-------------------
* Add boolean substitutions (`#598 <https://github.com/ros2/launch/issues/598>`_)
* Contributors: Kenji Miyake

0.22.0 (2022-03-28)
-------------------
* Support scoping environment variables (`#601 <https://github.com/ros2/launch/issues/601>`_)
* Contributors: Jacob Perron

0.21.1 (2022-03-01)
-------------------
* 'output' is expanded as a substitution in XML/YAML files (`#577 <https://github.com/ros2/launch/issues/577>`_)
* Contributors: Khush Jain

0.21.0 (2022-01-14)
-------------------

0.20.0 (2021-11-29)
-------------------
* Declare frontend group dependency & use explicit dependencies in launch_testing (`#520 <https://github.com/ros2/launch/issues/520>`_)
* Update maintainers to Aditya Pande and Michel Hidalgo (`#559 <https://github.com/ros2/launch/issues/559>`_)
* Updated maintainers (`#555 <https://github.com/ros2/launch/issues/555>`_)
* Add AppendEnvironmentVariable action (`#543 <https://github.com/ros2/launch/issues/543>`_)
* Feature clear launch configs (`#515 <https://github.com/ros2/launch/issues/515>`_)
* Fix `DeclareLaunchArgument` xml parsing and constructor (`#529 <https://github.com/ros2/launch/issues/529>`_)
* Add 'launch' to sets of launch file extensions (`#518 <https://github.com/ros2/launch/issues/518>`_)
* Contributors: Aditya Pande, Audrow Nash, Christophe Bedard, Derek Chopp, Ivan Santiago Paunovic

0.19.0 (2021-07-15)
-------------------
* Make each parser extension provide a set of file extensions (`#516 <https://github.com/ros2/launch/issues/516>`_)
* Contributors: Christophe Bedard

0.18.0 (2021-06-18)
-------------------

0.17.0 (2021-04-06)
-------------------

0.16.0 (2021-03-19)
-------------------

0.15.0 (2021-01-25)
-------------------

0.14.0 (2020-12-08)
-------------------
* Add frontend support for LogInfo action (`#467 <https://github.com/ros2/launch/issues/467>`_)
* Contributors: Jacob Perron

0.13.0 (2020-11-04)
-------------------
* Validate unparsed attributes and subentities in launch_xml and launch_yaml (`#468 <https://github.com/ros2/launch/issues/468>`_)
* Add test for launch.actions.TimerAction (`#470 <https://github.com/ros2/launch/issues/470>`_)
* Update package maintainers (`#465 <https://github.com/ros2/launch/issues/465>`_)
* Contributors: Ivan Santiago Paunovic, Michel Hidalgo

0.12.0 (2020-08-18)
-------------------

0.11.1 (2020-08-14)
-------------------

0.11.0 (2020-08-04)
-------------------
* Use new type_utils functions (`#438 <https://github.com/ros2/launch/issues/438>`_)
* Add pytest.ini so local tests don't display warning (`#428 <https://github.com/ros2/launch/issues/428>`_)
* Contributors: Chris Lalancette, Ivan Santiago Paunovic

0.10.2 (2020-05-26)
-------------------

0.10.1 (2020-05-08)
-------------------

0.10.0 (2020-04-24)
-------------------
* more verbose test_flake8 error messages (same as `ros2/launch_ros#135 <https://github.com/ros2/launch_ros/issues/135>`_)
* Use imperative mood in docstrings. (`#362 <https://github.com/ros2/launch/issues/362>`_)
* Contributors: Dirk Thomas, Steven! Ragnarök

0.9.5 (2019-11-13)
------------------

0.9.4 (2019-11-08)
------------------

0.9.3 (2019-10-23)
------------------

0.9.2 (2019-10-23)
------------------
* install resource marker file for packages (`#341 <https://github.com/ros2/launch/issues/341>`_)
* Contributors: Dirk Thomas

0.9.1 (2019-09-25)
------------------

0.9.0 (2019-09-18)
------------------
* install package manifest (`#330 <https://github.com/ros2/launch/issues/330>`_)
* Add deprecated argument to LaunchDescriptionn (`#291 <https://github.com/ros2/launch/issues/291>`_)
* Add support for not optional environment variable substitution (`#288 <https://github.com/ros2/launch/issues/288>`_)
* Add parsing methods for SetEnviromentVariable and UnsetEnviromentVariable (`#272 <https://github.com/ros2/launch/issues/272>`_)
* Add parsing method for `DeclareLaunchArgument` (`#270 <https://github.com/ros2/launch/issues/270>`_)
* Add frontend module in launch, launch_xml and launch_yaml packages (`#226 <https://github.com/ros2/launch/issues/226>`_)
* Contributors: Dirk Thomas, ivanpauno

