^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_dashboard
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.1 (2014-03-20)
------------------
* add rqt_gui_cpp dependency
* cob_battery for dashboard
* python catkin stuff
* merged catkin version
* Initial catkinization.
* update deps
* Modified dashboard to use rqt framework.
  Shamelessly adapted from rqt_pr2_dashboard.
* remove dependency to pr2_dashbord (deprecated)
* temporary blacklisted old cob_dashboard. needs to be migrated to rqt
* some cleaning up
* fixing deprecation of roslib.rosenv
* missing file
* emergency stop message: misused wireless field from power_board/state message as scanner stop filed
* removed unused files, files are used from original pr2_dashboard package
* removed Breakers from dashboard
* minimize cob_dashbaord, throw away Breaker and Motor buttons
* renamed last dashboard lines to command gui
* modified manifests
* changed name of cob_dashboard to cob_command_gui and added cob_dashboard overlay
* make nav buttons optional
* removed launch files
* added diagnostics aggregator
* new cob_command_tools stack
* analyzer for sdh
* added diagnostics aggregator
* added initAll and recoverAll buttons to dashboard
* added ENV variables to test
* added rostest
* Merge branch 'master' into syncmm
* merged with current master
* merged pull
* modified scrit server with modes for base movements
* added potential field nav to simplescriptserver
* small modifications on dashboard
* Merge branch 'syncmm' of github.com:abubeck/cob_apps into syncmm
* workaround for image size gtk bug
* Merge branch 'review-aub'
* Merge branch 'master' of github.com:abubeck/cob_apps into review-aub
* error handling for detect ojbect
* Merge branch 'master' into syncmm
* added switchable planning mode to dashboard, added cob_arm_navigation to cob_bringup for simulation
* added python api test for script server
* debugged trajectory calculation, working for multiple points
* moved launchfile
* delete dashboard parameters before uploading new ones
* removed config files from apps packages
* restructured dashboard to load robot and robot_env parts
* implemented points inside trajectories
* changed launchfile to use cob_default_config package
* fixed typo
* dashboard using cob_default_config package
* added support for multiple arms on the dashboard
* release update for cob3-1
* commit local changes
* added bringup with camera starting
* preparing release
* cleanup in cob_apps and updated stack.xml's
* cob3-1 grasp script modifications
* devs for cob3-2
* end of research-camp
* research camp challenge
* fixed bug with multiple notifications
* update dependency
* added emergency stop functionality
* update dependency
* added emergency stop functionality
* brics exercise 3 working again
* new arm transformation for lbr, set_operation_mode with service interface
* setting modes in dashboard
* removed init all button
* restructured base_controller
* modifications for cob3-1
* allow multiple instances of dashboard
* modified buttons for dashboard
* cleanup buttons in dashboard
* update urdf to be compatible with ctrutle, add 64bit support for libntcan
* merge
* update positions for lbr
* update documentation
* source documentation for cob_powercube_chain and cob_sdh
* modified names
* sdh changes and calibration script and parameter
* merged positions for lbr4
* added additional positions for lbr4
* modified urdf and adapted xaml files
* modification for cob3-2
* Merge branch 'master' of github.com:ipa-uh/care-o-bot
* modification on cob3-2
* temp from reza
* Merge branch 'master' of github.com:ipa-uh/care-o-bot
* theo told me to
* bugfix for script_server
* knoeppkes
* improved simulation for schunk arm and cleanup in 2dnav package
* fixed init bug
* removed tk code
* update dashboard
* merged older knoeppkes
* bugfix in dashboard
* dashboard launch file
* dashboard working with script_server
* added icons to dashboard
* dashboard gone gtk
* added support for mutiple esd dongles in sdh driver, changed dashboard for new lbr interfaces
* Merge branch 'cpc-pk' of git@github.com:ipa-cpc/care-o-bot into review-cpc-pk
* Added a action server to cob_camera_axis, tested successful with dashboard, recalibrated joint_head_eyes in cob3-1_torso.urdf
* moved script_server to open-source repository
* stop and init for lbr
* Merge branch 'master' of git@github.com:ipa-fmw/care-o-bot
* changed sdh interface to joint_trajectory_action
* improvements of lbr simulation
* added lbr to simulation
* added init for base
* updated simulation files
* cleaning up in cob_apps stack
* changes on powercube chain to accept direct command without actionlib
* modified for direct command
* added more buttons for arm movements
* modified trajectories
* modifications to knoeppkes
* new simulation interfaces
* big changes to simulation structure
* removed logout
* modifeid buttons
* modified knoeppkes
* adaptions to urdf for tray
* added buttons for lbr
* solo launch files
* implemented asynchron calls of buttons
* expanded knoeppkes to serve arm, tray, torso and sdh
* renamed cob launch file
* added torso buttons
* changes to pr2_controllers_msgs
* changed to pr2_controllers_msgs
* adaptions to gui
* adaptions to gui
* modified for pr2_arm simulation
* knoeppkes for arm is working
* new torso trajectory actions
* moved to cob_
* adapt launch file to new packages names
* renamed packages to cob_ convention
* Contributors: Alexander Bubeck, COB3-Manipulation, Georg Arbeiter, Michael Bowler, Philipp, Your full name, abubeck, b-it-bots-secure, fmw-br, fmw-jiehou, ipa-cob3-3, ipa-fmw, ipa-fxm, ipa-taj-dm, ipa-uhr-fm, uh
