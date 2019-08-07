^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_monitoring
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

0.6.13 (2019-07-19)
------------------
* Merge pull request `#248 <https://github.com/ipa320/cob_command_tools/issues/248>`_ from LoyVanBeek/feature/reduce_load
  Feature/reduce load
* PEP-8 this code
* Also use self.read_sysinfo in get_sys_net
* Cache file-handles instead of opening them again over and over again
  Reduces load om my laptop from 1.9% CPU to 1.1% CPU (after an initial warm up-period of < 20sec with higher usage)
* If value is str 'n/a', do not try to convert and calc with it, just use 0
* Instead of opening subprocess to cat info about network, open the file directly
  Reduces load from 6-7% to under 2%
* Instead of opening a subprocess to cat something, just open the file and read the contents
  Reduces load from this node on my system from 11-12% to 6-7%
* Contributors: Felix Messmer, Florian Weisshardt, Loy van Beek, andreeatulbure, fmessmer

0.6.12 (2019-06-07)
-------------------

0.6.11 (2019-04-05)
-------------------
* Merge pull request `#243 <https://github.com/ipa320/cob_command_tools/issues/243>`_ from KITrobotics/cpu_monitor_str_repair
  Repaired call of str object in cpu_monitor
* Merge pull request `#244 <https://github.com/ipa320/cob_command_tools/issues/244>`_ from fmessmer/enhance_wlan_monitor
  query all wireless interfaces, fix parsing
* query all wireless interfaces, fix parsing
* Repaired call of str object in cpu_monitor
* Contributors: Felix Messmer, andreeatulbure, fmessmer

0.6.10 (2019-03-14)
-------------------
* Merge pull request `#242 <https://github.com/ipa320/cob_command_tools/issues/242>`_ from fmessmer/diagnostics_based_em_stop_monitor
  diagnostics-based emergency state verbalization
* diagnostics-based emergency state verbalization
* Merge pull request `#236 <https://github.com/ipa320/cob_command_tools/issues/236>`_ from fmessmer/missing_dependency_python-requests
  add missing dependency python-requests
* add missing dependency python-requests
* add missing rosdep key ifstat
* Merge pull request `#235 <https://github.com/ipa320/cob_command_tools/issues/235>`_ from fmessmer/network_monitor_internal
  network monitor internal
* additional net and statistic keys
* proper timer and STALE handling
* add net_monitor from ethz-asl/ros-system-monitor
* Merge pull request `#232 <https://github.com/ipa320/cob_command_tools/issues/232>`_ from Acuda/feature/core_thermal_throttling
  new metrics (thermal throttling, idlejitter) for cpu monitor based on netdata
* use False as default in order to not produce stale/error diagnostics for robots that do not want/have the respective tools setup
* new metrics (thermal throttling, idlejitter) for cpu monitor based on netdata
* Contributors: Björn Eistel, Felix Messmer, fmessmer

0.6.9 (2018-07-21)
------------------
* update maintainer
* Contributors: ipa-fxm

0.6.8 (2018-07-21)
------------------
* Merge pull request `#205 <https://github.com/ipa320/cob_command_tools/issues/205>`_ from fmessmer/wlan_monitor
  adding wlan_monitor
* fix syntax
* do not set diagnostic level on high core temperature
* allow ssh connection without passwd via ssh-key
* humanreadable exceptions
* adding wlan_monitor
* Merge pull request `#223 <https://github.com/ipa320/cob_command_tools/issues/223>`_ from fmessmer/fix_monitor_virtual_temp
  refactor monitors
* call update once at startup to prevent No Data diagnostics
* more monitoring info
* fix nodes on robot
* cleanup, consistency and proper timer
* more info, more robust, more consistent
* only get temperatures of platform devices - no virtual
* Merge pull request `#221 <https://github.com/ipa320/cob_command_tools/issues/221>`_ from fmessmer/missing_dependency_ntpdate
  add missing dependency ntpdate
* add missing dependency ntpdate
* Contributors: Felix Messmer, ipa-fxm, robot@cob4-15

0.6.7 (2018-01-07)
------------------
* Merge remote-tracking branch 'origin/indigo_release_candidate' into indigo_dev
* Merge pull request `#214 <https://github.com/ipa320/cob_command_tools/issues/214>`_ from ipa-fmw/fix/emstop_monitor
  [EM stop monitor] prevent emstop monitor from saying empty strings
* prevent emstop monitor from saying empty strings
* Merge pull request `#211 <https://github.com/ipa320/cob_command_tools/issues/211>`_ from ipa-fxm/enhance_em_sound_logic
  enhance emergency sound output
* enhance emergency sound output
* Merge pull request `#208 <https://github.com/ipa320/cob_command_tools/issues/208>`_ from ipa-fxm/allow_distinct_say_on_release
  allow distinct say on released
* allow distinct say on released
* Merge pull request `#207 <https://github.com/ipa320/cob_command_tools/issues/207>`_ from ipa-fxm/sound_emergency_stop_monitor
  Sound emergency stop monitor
* allow to configure battery monitor notifications
* allow to configure emergency stop notifications
* Merge pull request `#200 <https://github.com/ipa320/cob_command_tools/issues/200>`_ from ipa-fxm/configurable_ntp_monitor
  enhance ntp_monitor
* Merge pull request `#202 <https://github.com/ipa320/cob_command_tools/issues/202>`_ from ipa-fxm/update_maintainer
  update maintainer
* update maintainer
* refactor ntp_monitor
* made ntp_monitor configurable via yaml
* Merge pull request `#197 <https://github.com/ipa320/cob_command_tools/issues/197>`_ from ipa-fxm/APACHE_license
  use license apache 2.0
* use license apache 2.0
* Contributors: Felix Messmer, Florian Weisshardt, Richard Bormann, ipa-fxm, ipa-uhr-mk, robot@cob4-2

0.6.6 (2017-07-17)
------------------
* fix parameter name
* use desired frequency as default windows size for the topic freq monitor
* adjust tolerance, window_size
* allow to monitor multiple topics
* adjust to hz_monitor yaml layout
* generic topic status monitor based on diagnostic updater
* [hotfix] python syntax
* remove useless code (`#173 <https://github.com/ipa320/cob_command_tools/issues/173>`_)
* update psutil api
* add fake_diagnostics
* get num_cores from psutils
* add proper print_functions import
* some python3 print fixes
* rospy.sleep exception handling
* manually fix changelog
* make smapling rate configurable and add warning
* Contributors: Benjamin Maidel, Felix Messmer, Sourav Senapati, ipa-fxm, msh

0.6.5 (2016-10-10)
------------------
* fix diagnostics output
* - Made changes to handle HZ monitoring for multiple topics.
* - Minor fix for publishing multiple topics.
* - Made changes for handling multiple hz topics.
* - Completely commented out the code segments for HDD temperature statistics.
  - Added cla for providing directory name, default is root directory.
* use reasonable default window size and fix status level
* fix code style
* add hz monitor
* simplify emergency_stop monitor
* fix emergency_stop monitor for enable_light set to false
* enable cpu warnings in diagnostics
* increased battery_monitors led prio
* check if light is enabled
* init light mode object
* Contributors: Benjamin Maidel, Florian Weisshardt, fmw-hb, fmw-ss, ipa-cob4-5, ipa-nhg, msh

0.6.4 (2016-04-01)
------------------
* stop charging mode if no more power_state msgs received
* fix node and class name
* fix emergency_stop_monitor
* parameter name consistency
* fix script
* configurable battery thresholds
* parameter for enabling sound and light
* combine battery_light_monitor and battery_monitor
* add say output to battery_light_monitor
* added actionlib exec dep and install tag
* fixes
* fix
* fix
* use cob_lights track_id in battery light monitor
* adapted em stop monitor to new cob_light
* fixes due to cob_light changes
* changes due to cob_lights refactor
* implemented compatibility for non addressable led bands
* switched from info to debug message
* switched from action to service
* added monitor to switch cobs light if charging
* set queue size to 1
* Update emergency_stop_monitor.py
* fixed em stop monitor
* removed configuration files
* fixes type conversion in ddwrt
* Contributors: Benjamin Maidel, Florian Weisshardt, ipa-bnm, ipa-cob4-2, ipa-fxm, ipa-nhg

0.6.3 (2015-08-25)
------------------
* remove obsolete autogenerated mainpage.dox files
* remove trailing whitespaces
* remove trailing whitespaces
* migrate to package format 2
* sort dependencies
* critically review dependencies
* Contributors: ipa-fxm

0.6.2 (2015-06-17)
------------------
* fix emergency_stop_monitor (tested on cob4-2: OK)
* enhance emergency_stop_monitor with diagnostics_based and motion_based
* emergency stop monitor includes diagnostics and em stop
* reworked emergency_stop_monitor (sets leds based on diagnostics), still needs to be updated to be robot independent (hardcoded components)
* cleanup CMakeLists
* have speach output for emergency switch to OK
* make colors for error, warning and ok configurable
* fix light for simple_script_server, adapt emergency_stop_monitor for cob4 by supporting mulitple light components
* added install tags
* Contributors: Florian Weisshardt, ipa-cob4-2, ipa-fmw, ipa-fxm, ipa-nhg

0.6.1 (2014-12-15)
------------------
* Update battery_monitor.py
* move cob_monitoring to cob_command_tools
* Contributors: Florian Weisshardt, ipa-nhg

0.5.2 (2014-03-27)
------------------

0.5.1 (2014-03-20)
------------------
* Initial catkinization.
* no speach output for first emergency change
* enhanced battery monitoring
* separate monitoring
* add todos to monitoring
* add sound to em monitoring
* monitoring package
* Contributors: abubeck, ipa-fmw
