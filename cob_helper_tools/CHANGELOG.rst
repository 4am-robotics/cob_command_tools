^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_helper_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.16 (2020-03-18)
-------------------
* Merge pull request `#270 <https://github.com/ipa320/cob_command_tools/issues/270>`_ from LoyVanBeek/feature/python3_compatibility
  [ci_updates] pylint + Python3 compatibility
* python3 compatibility via 2to3
* Merge pull request `#271 <https://github.com/ipa320/cob_command_tools/issues/271>`_ from fmessmer/ci_updates
  [travis] ci updates
* catkin_lint fixes
* Merge pull request `#265 <https://github.com/ipa320/cob_command_tools/issues/265>`_ from HannesBachter/feature/recover_service
  add services to en- and disable auto recover
* add services to en- and disable auto recover
  add services to en- and disable auto recover
* Contributors: Felix Messmer, Loy van Beek, fmessmer, hyb

0.6.15 (2019-11-07)
-------------------
* Merge pull request `#253 <https://github.com/ipa320/cob_command_tools/issues/253>`_ from benmaidel/fix/visualize_nav_goals
  visualize nav_goals improvements
* improvements:
  - add every nav_goal to a separate namespace (enable/disable visual)
  - fix scaling for Arrow marker
* Contributors: Benjamin Maidel, Florian Weisshardt

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
* Merge pull request `#239 <https://github.com/ipa320/cob_command_tools/issues/239>`_ from fmessmer/max_retry_auto_init
  introduce max_retries for auto_init
* introduce max_retries for auto_init
* Merge pull request `#231 <https://github.com/ipa320/cob_command_tools/issues/231>`_ from fmessmer/auto_recover_diagnostics_based
  fix typo
* fix typo
* Merge pull request `#230 <https://github.com/ipa320/cob_command_tools/issues/230>`_ from fmessmer/auto_recover_diagnostics_based
  do not auto-recover based on diagnostics
* do not auto-recover based on diagnostics
* Merge pull request `#229 <https://github.com/ipa320/cob_command_tools/issues/229>`_ from HannesBachter/fix/auto_recover
  case insensitive comparison
* case insensitive comparison
* Contributors: Felix Messmer, fmessmer, hyb

0.6.9 (2018-07-21)
------------------
* update maintainer
* Contributors: ipa-fxm

0.6.8 (2018-07-21)
------------------
* Merge pull request `#217 <https://github.com/ipa320/cob_command_tools/issues/217>`_ from HannesBachter/feature/textsize_navgoal
  enable setting textsize for visualization of navigation goals
* use dynamic reconfigure for text size
* enable setting textsize of navigation goals
* Contributors: Richard Bormann, hyb, ipa-fxm

0.6.7 (2018-01-07)
------------------
* Merge remote-tracking branch 'origin/indigo_release_candidate' into indigo_dev
* Merge pull request `#213 <https://github.com/ipa320/cob_command_tools/issues/213>`_ from ipa-fxm/no_recover_em_stop
  do not recover on em_stop
* do not recover on em_stop
* Merge pull request `#212 <https://github.com/ipa320/cob_command_tools/issues/212>`_ from ipa-fxm/enhance_auto_recover_logic
  enhance auto_recover logic
* enhance auto_recover logic
* Merge pull request `#197 <https://github.com/ipa320/cob_command_tools/issues/197>`_ from ipa-fxm/APACHE_license
  use license apache 2.0
* use license apache 2.0
* Contributors: Felix Messmer, Florian Weisshardt, ipa-fxm, ipa-uhr-mk

0.6.6 (2017-07-17)
------------------
* move visualize navigation goals to cob_helper_tools
* retry init on failure
* only store timestamp for last recover on success
* add fake_diagnostics
* add fake_driver
* added license header
* evaluate handle and better output
* add auto_init
* add auto_recover to new cob_helper_tools pkg
* Contributors: Florian Weisshardt, ipa-fxm, robot
