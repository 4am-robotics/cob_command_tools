^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package generic_throttle
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
