^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_helper_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.30 (2022-11-17)
-------------------

0.6.29 (2022-07-29)
-------------------
* Merge pull request `#319 <https://github.com/ipa320/cob_command_tools/issues/319>`_ from fmessmer/feature/extend_fake_driver
  proper service and diagnostic logic for fake_driver
* proper service and diagnostic logic for fake_driver
* Contributors: Felix Messmer, fmessmer

0.6.28 (2022-03-15)
-------------------
* Merge pull request `#310 <https://github.com/ipa320/cob_command_tools/issues/310>`_ from HannesBachter/fix/sleep_init
  sleep for 1 second after unsuccessful try to initialize
* sleep for 1 second after unsuccessful try to initialize
* Contributors: Felix Messmer, HannesBachter

0.6.27 (2022-01-12)
-------------------
* Merge pull request `#309 <https://github.com/ipa320/cob_command_tools/issues/309>`_ from pgehring/feature/auto_tools
  combine auto_init and auto_recover
* check enable state for subscribing and unsubscribing
* fix imports
* fix package name conflict
* Revert "move dynamic reconfigure to python module"
  This reverts commit ce13c73a7638b44a8dcf23f75c82d86a83431ad4.
* move dynamic reconfigure to python module
* preserve auto_init and auto_recover nodes
* use params from server to enable init or recover
* setup auto_tools node
* add auto_tools node
* move auto_init and auto_recover to python module
* Contributors: Felix Messmer, pgehring

0.6.26 (2021-11-26)
-------------------

0.6.25 (2021-08-02)
-------------------

0.6.24 (2021-07-02)
-------------------
* Merge pull request `#304 <https://github.com/ipa320/cob_command_tools/issues/304>`_ from fmessmer/fixup_auto_tools
  fixup condition in auto_recover
* fixup condition in auto_recover
* Contributors: Felix Messmer, fmessmer

0.6.23 (2021-07-01)
-------------------
* Merge pull request `#303 <https://github.com/ipa320/cob_command_tools/issues/303>`_ from fmessmer/enhance_auto_tools
  enhance auto tools
* tmp: debug logoutput for auto_recover for disabled cases
* introduce em_state_ignore for auto_init
* allow infinite auto_init
* use message enum
* Contributors: Felix Messmer, fmessmer

0.6.22 (2021-05-10)
-------------------

0.6.21 (2021-04-06)
-------------------
* Merge pull request `#301 <https://github.com/ipa320/cob_command_tools/issues/301>`_ from HannesBachter/fix/visualize_in_map
  remove leading / in frame_id
* remove leading / in frame_id
* Contributors: Felix Messmer, HannesBachter

0.6.20 (2021-01-25)
-------------------

0.6.19 (2020-12-02)
-------------------
* Merge pull request `#287 <https://github.com/ipa320/cob_command_tools/issues/287>`_ from fmessmer/fix_catkin_lint
  fix catkin_lint
* fix catkin_lint
* Contributors: Felix Messmer, fmessmer

0.6.18 (2020-10-21)
-------------------

0.6.17 (2020-10-17)
-------------------
* Merge pull request `#284 <https://github.com/ipa320/cob_command_tools/issues/284>`_ from fmessmer/test_noetic
  test noetic
* Bump CMake version to avoid CMP0048 warning
* Contributors: Felix Messmer, fmessmer

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
