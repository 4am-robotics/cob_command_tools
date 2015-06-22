^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_monitoring
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.3 (2014-03-28)
------------------

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
