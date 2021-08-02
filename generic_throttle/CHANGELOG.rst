^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package generic_throttle
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

0.6.24 (2021-07-02)
-------------------

0.6.23 (2021-07-01)
-------------------

0.6.22 (2021-05-10)
-------------------

0.6.21 (2021-04-06)
-------------------

0.6.20 (2021-01-25)
-------------------
* Merge pull request `#294 <https://github.com/ipa320/cob_command_tools/issues/294>`_ from fmessmer/fix_python3
  fix python3
* fix python3
* Merge pull request `#291 <https://github.com/ipa320/cob_command_tools/issues/291>`_ from fmessmer/fix_python3
  fix python3
* Merge pull request `#3 <https://github.com/ipa320/cob_command_tools/issues/3>`_ from LoyVanBeek/fix_python3
  Get first key and value compatible with both Python 2 & 3
* Get first key and value compatible with both Python 2 & 3
* Contributors: Felix Messmer, Loy van Beek, fmessmer

0.6.19 (2020-12-02)
-------------------

0.6.18 (2020-10-21)
-------------------

0.6.17 (2020-10-17)
-------------------
* Merge pull request `#284 <https://github.com/ipa320/cob_command_tools/issues/284>`_ from fmessmer/test_noetic
  test noetic
* use setuptools instead of distutils
* Bump CMake version to avoid CMP0048 warning
* Contributors: Felix Messmer, fmessmer

0.6.16 (2020-03-18)
-------------------
* Merge pull request `#270 <https://github.com/ipa320/cob_command_tools/issues/270>`_ from LoyVanBeek/feature/python3_compatibility
  [ci_updates] pylint + Python3 compatibility
* fix more pylint errors
* fix pylint errors
* Merge pull request `#271 <https://github.com/ipa320/cob_command_tools/issues/271>`_ from fmessmer/ci_updates
  [travis] ci updates
* catkin_lint fixes
* Contributors: Felix Messmer, fmessmer

0.6.15 (2019-11-07)
-------------------

0.6.14 (2019-08-07)
-------------------

0.6.13 (2019-07-19)
------------------

0.6.12 (2019-06-07)
-------------------

0.6.11 (2019-04-05)
-------------------

0.6.10 (2019-03-14)
-------------------

0.6.9 (2018-07-21)
------------------
* update maintainer
* Contributors: ipa-fxm

0.6.8 (2018-07-21)
------------------

0.6.7 (2018-01-07)
------------------
* Merge remote-tracking branch 'origin/indigo_release_candidate' into indigo_dev
* Merge pull request `#209 <https://github.com/ipa320/cob_command_tools/issues/209>`_ from ipa-fxm/generic_throttle_private_param
  use private parameters for generic_throttle
* use private parameters for generic_throttle
* Merge pull request `#197 <https://github.com/ipa320/cob_command_tools/issues/197>`_ from ipa-fxm/APACHE_license
  use license apache 2.0
* use license apache 2.0
* Contributors: Benjamin Maidel, Felix Messmer, ipa-fxm, ipa-uhr-mk

0.6.6 (2017-07-17)
------------------
* Change resolution factor to be a multiplicative factor and not a division.
  Handle resize to factor 0 as OpenCV error.
* Change misleading resolution with resolution_factor
* Add rostopic exec_depend
* Remove cv_bridge build dependency
* Update manifest and cmake list
* Update README.md to resolution throttle.
* Implement resolution throttle for sensor_msgs/Image.
* Add @ipa-fmw as maintainer.
* Add install tags for the executable node
* Move throttle_node to script folder. Fix typos in README.
* Update README
* Rewrite lazy and latched behavior implementation
* Adapt to new parameter definition layout. Parameters are now set in the node namespace.
  Throttled topics have "_throttled" appended to original topic name.
* Add ROS node for the throttle separated from the GenericThrottle implementation
* Implement lazy behavior. Lazy and Latched behavior are disabled by default
* Remove delay feature. Add latched behaviour (True/False).
* Remove build dependency on rospy
* Remove CMakeLists.txt not needed depedencies.
* Change package to format "2" and remove not needed dependencies
* Update README
* Moving the python package inside src/generic_throttle. Modifying accordingly CMakeLists.txt and package.xml.
* remove testing files
* introduce directory layer
* Contributors: MattiaRacca
