^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_script_server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.16 (2020-03-18)
-------------------
* Merge pull request `#280 <https://github.com/ipa320/cob_command_tools/issues/280>`_ from fmessmer/fix/init_urdf_structure
  initialize urdf structure only once during init
* initialize urdf structure only once during init
* Merge pull request `#270 <https://github.com/ipa320/cob_command_tools/issues/270>`_ from LoyVanBeek/feature/python3_compatibility
  [ci_updates] pylint + Python3 compatibility
* Use six to handle input vs raw_input
* cleanup cob_script_server
* fix more pylint errors
* use threading
* fix pylint errors
* python3 compatibility via 2to3
* Merge pull request `#273 <https://github.com/ipa320/cob_command_tools/issues/273>`_ from floweisshardt/fix/move_base_without_global_map
  publish move_base goal without global /map
* publish move_base goal without global /map
* Merge pull request `#271 <https://github.com/ipa320/cob_command_tools/issues/271>`_ from fmessmer/ci_updates
  [travis] ci updates
* catkin_lint fixes
* Merge pull request `#260 <https://github.com/ipa320/cob_command_tools/issues/260>`_ from fmessmer/feature/calculate_traj_point_velocities
  proper traj velocity calculation
* add opt-out stop_at_waypoints
* use logdebug for debug output
* fix trajectory point velocity calculation
* drop start after time calculation
* filter duplicate or close configs in trajectory
* fix array type
* set accelerations to 0.0
* not use min_point_time 0.4
* drop first traj point if close to start_pos
* improve debug output
* remove hardcoded point velocities and accelerations
* add debug output
* epsilon check and elementwise zero-velocity
* proper traj velocity calculation
* Merge pull request `#264 <https://github.com/ipa320/cob_command_tools/issues/264>`_ from fmessmer/fix/robot_description_ns
  fix robot_description namespace handling
* fix robot_description namespace handling
* Merge pull request `#259 <https://github.com/ipa320/cob_command_tools/issues/259>`_ from LoyVanBeek/feature/faster_sss_trajectories
  Feature/faster sss trajectories
* fix log formatting
* explicit robot_description key
* Move all code to determine the desired velocity to a private method
  Code to determine this got too big to contain all the various branches and edge cases
* Fix undeclared variable
* feature/calculate traj point velocities (`#1 <https://github.com/ipa320/cob_command_tools/issues/1>`_)
  feature/calculate traj point velocities
* fix error case velocities zero or negative
* fixup acc default value
* fixup variable name
* fixup limit_vel and add debug output
* add vel_limit check
* return failure on invalid arguments
* expose arguments to move_rel
* allow to pass default_vel via sss argument
* allow setting default_vel and default_acc per joint as list of float/int via yaml
* Rename d_max to dist
* Optionally use the URDF-derived velocities
* Extend sss.move with a speed_factor to speed up movements
* fixup! Iterate over all joints to calculate_point_time and take the  slowest time for all joints
* Get joint velocity from URDF in compose_trajectory
* Iterate over all joints to calculate_point_time and take the  slowest time for all joints
  Earlier only 1 joint velocity and acceleration was used.
  TOOD: Read velocity (and acceleration?) from URDF
* Explicitly import numpy. This was implicitly imported via at least tf.transformations and possibly others
* Contributors: Felix Messmer, Florian Weisshardt, Loy, Loy van Beek, floweisshardt, fmessmer

0.6.15 (2019-11-07)
-------------------
* Merge pull request `#261 <https://github.com/ipa320/cob_command_tools/issues/261>`_ from fmessmer/fix/travis
  fix/travis
* disable test
* Merge pull request `#256 <https://github.com/ipa320/cob_command_tools/issues/256>`_ from floweisshardt/fix/joint_order
  fix joint order in sss
* fix joint order
* Merge pull request `#250 <https://github.com/ipa320/cob_command_tools/issues/250>`_ from HannesBachter/feature/move_rel
  Feature/move rel
* fix ah handling and dependency
* return move_traj action_handle
* calculate poses for multiple movements
* add urdf, cleanup and return in error case
* check parameters
* move rel with check of joint limits
* Contributors: Felix Messmer, Florian Weisshardt, floweisshardt, fmessmer, hyb, robot@mrl-a

0.6.14 (2019-08-07)
-------------------

0.6.13 (2019-07-19)
------------------

0.6.12 (2019-06-07)
-------------------
* Merge pull request `#247 <https://github.com/ipa320/cob_command_tools/issues/247>`_ from fmessmer/fix_variable_overflow_serialization_exception
  replace ScriptState number with datetime for uniqueness
* replace ScriptState number with datetime for uniqueness
* Contributors: Felix Messmer, fmessmer

0.6.11 (2019-04-05)
-------------------

0.6.10 (2019-03-14)
-------------------
* Merge pull request `#241 <https://github.com/ipa320/cob_command_tools/issues/241>`_ from fmessmer/add_string_action
  Add string action
* add SetString action interface
* temp_woz
* Merge pull request `#237 <https://github.com/ipa320/cob_command_tools/issues/237>`_ from fmessmer/default_value_handling
  fix comments for default values
* fix comments for default values
* Merge pull request `#227 <https://github.com/ipa320/cob_command_tools/issues/227>`_ from floweisshardt/feature/new_trajectory
  new trajectory point time calculation
* remove print
* fix syntax
* new trajectory point time calculation
* Contributors: Felix Messmer, Florian Weisshardt, fmessmer, ipa-fmw, robot@cob4-19

0.6.9 (2018-07-21)
------------------
* update maintainer
* Contributors: ipa-fxm

0.6.8 (2018-07-21)
------------------
* Merge pull request `#220 <https://github.com/ipa320/cob_command_tools/issues/220>`_ from ipa320/revert-219-restructure_sss_params
  Revert "use joint_names from component ns"
* Revert "use joint_names from component ns"
* Merge pull request `#219 <https://github.com/ipa320/cob_command_tools/issues/219>`_ from ipa-fxm/restructure_sss_params
  use joint_names from component ns
* use joint_names from component ns
* Contributors: Felix Messmer, ipa-fxm

0.6.7 (2018-01-07)
------------------
* Merge remote-tracking branch 'origin/indigo_release_candidate' into indigo_dev
* Merge pull request `#206 <https://github.com/ipa320/cob_command_tools/issues/206>`_ from ipa-fmw/hotfix_cob_console
  [cob_console] hotfix for ipython shebang
* hotfix for cob_console ipython shebang
* Merge pull request `#197 <https://github.com/ipa320/cob_command_tools/issues/197>`_ from ipa-fxm/APACHE_license
  use license apache 2.0
* use license apache 2.0
* Contributors: Felix Messmer, Florian Weisshardt, ipa-fxm, ipa-uhr-mk

0.6.6 (2017-07-17)
------------------
* move visualize navigation goals to cob_helper_tools
* add node for visualization of script server navigation goals
* handle uninitielized action clients
* add trigger_action function to sss
* fix Trigger unittests
* increased min point time
* minimal trajectory point_time
* catch ValueError during point time calculation when using mimic joints
* manually fix changelog
* fixed cob_console script
* Contributors: Benjamin Maidel, Florian Weisshardt, Mathias Lüdtke, ipa-fxm

0.6.5 (2016-10-10)
------------------
* use joint_states instead of controller_state
* Removed an extra space
* Removed a comma
* fixed more concatenate messages errors
* Update log message
* Fix concatenate error
* fix error message for move_base_rel
* Contributors: Florian Weisshardt, Nadia Hammoudeh García, ipa-cob4-5, ipa-fmw, ipa-nhg

0.6.4 (2016-04-01)
------------------
* undo pop-up related changes
* indentation fixes
* more verbose action_handle (takes an additional string in set_failed())
  more consistent usage of action_handle throughout code
* cleanup action_handle
* parameter for enabling sound and light
* allow passing of component_name to sss.say
* use modes definition instead of magic numbers
* Merge branch 'indigo_dev' into fix/refactor_light
  Conflicts:
  cob_teleop/ros/src/cob_teleop.cpp
* reduce timeout for all wait_for_server calls
* fixes due to cob_light changes
* shorten timeout
* fix CMakeLists.txt
* add dependency to actionlib
* fix ScriptAction include in cob_teleop
* fix sss error handling for light
* Merge branch 'fix_teleop' into fix_sss
* set zero velocity and acceleration for all trajectory points
* fix actionhandle for trigger
* deleted script_server tests from launch file
* Contributors: Benjamin Maidel, Florian Köhler, Florian Weisshardt, ipa-cob4-2, ipa-fmw, ipa-fxm

0.6.3 (2015-08-25)
------------------
* do not install headers in executable-only packages
* more cleanup
* remove obsolete autogenerated mainpage.dox files
* remove trailing whitespaces
* remove trailing whitespaces
* migrate to package format 2
* Merge pull request `#105 <https://github.com/ipa-fxm/cob_command_tools/issues/105>`_ from ipa-nhg/play_sound
  minor changes
* minor changes
* Merge pull request `#103 <https://github.com/ipa-fxm/cob_command_tools/issues/103>`_ from ipa-nhg/play_sound
  play sound
* log error
* cob_sound
* sort dependencies
* critically review dependencies
* play sound
* Contributors: Florian Weisshardt, ipa-fxm, ipa-nhg

0.6.2 (2015-06-17)
------------------
* merge with ipa320
* Merge pull request `#18 <https://github.com/ipa320/cob_command_tools/issues/18>`_ from ipa-cob4-2/indigo_dev
  updates from cob4-2
* use actions for light, sound and mimic. Using new namespaces with component_name
* use wait_for_message instead of joint_state_listener
* use new Trigger from std_srvs
* fix indention
* fix wrong service handle
* Merge branch 'indigo_dev' of https://github.com/ipa-fmw/cob_command_tools into indigo_dev
* fix blocking of move_base_rel and add mimic support
* fix wrong variable name
* fixed bug: light service is expecting 4 instead of 3 parameters [r,g,b,a]
* cleanup CMakeLists
* fix light for simple_script_server, adapt emergency_stop_monitor for cob4 by supporting mulitple light components
* use transparency parameter, tiomeout for service and tabs vs spaces
* using light service instead of topic and adapted for multiple components
* added topic_name parameter set_light
* Contributors: Benjamin Maidel, Florian Weisshardt, ipa-cob3-9, ipa-cob4-2, ipa-fmw, ipa-fxm, ipa-nhg

0.6.1 (2014-12-15)
------------------
* fix traj time calculation
* use default vel instead of default point time
* use 8 sec by default for trajectories
* action and service namespaces are configurable now
* add halt service support
* Missing install tag
* adapt namespaces to new canopen version
* Contributors: Florian Weisshardt, ipa-cob4-2, ipa-fmw

0.6.0 (2014-09-18)
------------------

0.5.2 (2014-08-28)
------------------
* missing dependency
* added explicit default argument queue_size
* fix catkin_lint errors
* add trajectory service
* add dep to ipython
* Update package.xml
* Contributors: Felix Messmer, Florian Weisshardt, ipa-fmw-ms, ipa-fxm, ipa-jenkins, ros

0.5.1 (2014-03-20)
------------------
* Install tags
* removed a lot of code related to packages not available in hydro anymore
* deactivate tests
* needed for python import
* removed old scriptserver location
* changed location of script server for installation
* remove arm navigation stuff, now using moveit anyway
* python catkin stuff
* merged catkin version
* Initial catkinization.
* removing some earlier commits 3
* removing some earlier commits 2
* removing some earlier commits
* fix cob_console
* removed blocking from Script.action because blocking or non-blocking behaviour can be specified by actionlib directly
* set blocking to True by default
* change test script
* removed deprecated scripts
* Modify execute_cb in script_server to support any function in sss
  Add blocking, service_name, duration and planning to Script.action to support script_server change
  return ah from sleep() in sss
* Modify execute_cb in script_server to support any function in sss
  Add blocking, service_name, duration and planning to Script.action to support script_server change
  return ah from sleep() in sss
* add cob_console
* updated test_script
* Revert "changed component names to explicitly contain full namespace"
  This reverts commit b3cf8a5e500a754d19091aba25a9fe442518556d.
* Merge branch 'master' of github.com:ipa-fmw/cob_command_tools
* changed component names to explicitly contain full namespace
* fix action handle for light
* Merge remote-tracking branch 'origin-ipa320/master' into automerge
* switched from pr2_controllers_msgs::JointTrajectoryAction to control_msgs::FollowJointTrajectory
* removed functions used for cartesian motion and ik
* unified script_server: removed all functions related to planned cartesian motion for the arm + some fixes
* removed unecessary code
* fix action result for play
* fixed issues with action handle state for non actionlib functions
* use constraint_aware ik-solver; some minor modifications
* fixed typo
* use follow_joint_trajectory instead of joint_trajectory_action; fixed hardcoded length for velocities in trajectory points_msg
* removed call to set_planning_scene_diff in move_constrained_planned
* removed cartesian-related functions
* Merge branch 'master' of github.com:ipa320/cob_command_tools
* removed hard-code call to set_planning_scene_diff
* removed call to transfrom pose service
* Merge branch 'review-ipa320'
* fix typo
* changed light to std_msgs/ColorRGBA message
* fixed whitespace
* fixed calculate_ik
* fixed parse_cartesian_parameters
* disabled GetPoseStampedTransformed calls
* more output for move_planned_constraint
* fixed parse_cartesian_parameters
* introduced parse_cartesian_parameters
* Merge pull request `#1 <https://github.com/ipa320/cob_command_tools/issues/1>`_ from ipa-fmw-ws/master
  Check_plan im sss
* move_base_rel with safe topic
* Merge branch 'review-ipa320'
* changed to cartesian goal and start in joint space
* for testing check_plan function in sss
* removed dummy velocities
* Merge remote-tracking branch 'origin-ipa320/master' into automerge
* non blocking service calls working
* added support for setting manipulation velocity
* Added check_plan: Check if a trajectory to a specific goal exists
* test script for ik calculation and motion plan
* support for init_all and recover_all based on loaded robot modules in command_gui
* Merge branch 'master' of https://github.com/abubeck/cob_command_tools into review-abubeck
* Merge branch 'review-ipa320'
* TEST: added planning_scene_diff to MoveArmGoal for considering collision_objects during move_constrained_planned
* base stop wirking
* fix service stop
* added ah.cancel and stop for base
* changed follow joint trajectory name
* fixed dependency on pygraphviz
* updated dependencies
* moved GetPoseStampedTransformed.srv to cob_srvs
* Merge branch 'master' of github.com:ipa320/cob_command_tools
* added initial version of move_cart_planned
* added component_name guard for _planned functions
* fixed indention error
* minor changes for HW tests
* intergrated pose transform service call
* fuerte rosdep migration
* changed script server to followjointtrajectory action
* introduced move_pose_goal_planned
* renamed move_planned to move_joint_goal_planned, keeping old name for compatibility reasons
* added move_constrained_planned, move_planned is now calling it
* read ik_link_name directly from /cob_arm_kinematics/arm/tip_name
* read joint names directly from /arm_controller/state
* modified calculate_ik to use solver from cob_arm_navigation
* adapt roslaunch tests
* more informative error messages
* merge conflict
* added relative motion to script server
* bugfix
* add sss.calculate_ik for ik pre calculation and therefore removed move_cart
* test for script server
* remove hack
* new cob_command_tools stack
* fix random moves
* ros navigation working mostly fine
* added missing scriptserver functions
* remove compiler warnings
* removed failing test
* deaktivate test because fails on hudson
* longer timeout for tests
* disable move base omni test
* fix arm movements in script_server
* added ENV variables to tests
* start generic states
* fix navigation
* modified test
* beautify tests
* added actionlib tests
* added launch file checks
* new test files for cob_script_server
* switched to electric
* removed dependency to cob_msgs
* interaid adaptions
* fetch and carry on cob3-3
* say test
* changed script server details
* update script_server for linear base movement
* test cooler
* calibration scrit for cob3-1
* modified scrit server with modes for base movements
* merge
* added potential field nav to simplescriptserver
* added calibration script cob3-1
* testcooler
* calib script for cob3-3
* update for cob3-3
* solved merge conflict
* merge
* adapted china_cup initial position
* wimicare project: modifications
* remove sound_play from script_server
* commit after merge
* modifications for wimicare project
* removed detection section from simple_script_server
* implemented all object_handler functions
* start integrating object_handler to script_server
* Merge branch 'review-aub'
* Merge branch 'review-320'
* changed say interface to cob_sound
* cleanup script_server
* implementation of move_cart_planned in cob_script_server
* Merge branch 'master' of github.com:ipa-uhr-fm/cob_apps into review-uhr-fm
* merged with 320
* restructured cob_arm_navigation
* fix
* state checking while parsing
* change to executable mode
* Merge branch 'review-320'
* extended script server test
* back to 3 sec er movemement
* Merge branch 'review-320'
* reordered kitchen objects
* added autostart
* first version of ToF sensor data to collision map for dynamic environment
* merge ipa320
* merge with ipa320
* fixed colliding trajectories (tablet_padding)
* renamed move action to script action
* Merge branch 'review-uhr-fm'
* Merge branch 'review-aub'
* Merge branch 'master' of github.com:abubeck/cob_apps into review-aub
* monday evening commit
* Merge branch 'master' of https://github.com/ipa-uhr-fm/cob_apps into review-uhr-fm
* changed trajectory time to 10 seconds
* typo
* dep to actionlib_msgs
* Merge branch 'master' of https://github.com/ipa320/cob_apps into review-320
* update stacks
* fix
* error handling for detect ojbect
* Merge branch 'review-taj-dm'
* moved ultiple message files out of cob_msgs to their own packages
* get milk is working
* Merge branch 'review-320'
* integrated object detection into script server
* added detect object funtion
* added switchable planning mode to dashboard, added cob_arm_navigation to cob_bringup for simulation
* added all trigger tests
* added python api test for script server
* return handle for trigger commands
* integrated planning in script_server
* prepare script server for smach
* separate script server from action handle
* added test script for head joints
* added depencency
* Merge branch 'master' of github.com:ipa-uhr-fm/cob_apps into review-uhr-fm
* removed deprecated dependencies
* Merge branch 'master' of github.com:ipa320/cob_apps into ipa320-review
* implemented pause in script server
* script to graph working
* grasping china_cup is working
* merge with review-320
* removed config files from apps packages
* implemented points inside trajectories
* read joint_names from parameter server
* added platfrom test script
* changed launchfile to use cob_default_config package
* added support for multiple arms on the dashboard
* commit local changes
* added bringup with camera starting
* preparing release
* debugged service interface for gazebo
* partial merge with ipa-uhr-fm
* calibration script for neck-camera on cob3-1
* calibration script
* changed trigger service
* typo
* merge
* cleanup in cob_apps and updated stack.xml's
* Merge branch 'master' of github.com:ipa-fmw/care-o-bot into fmw-messmerf
  Conflicts:
  cob_apps/cob_arm_navigation/CMakeLists.txt
* joint_state aggregator working on cob3-1, calibration script update
* added head
* calib script for cob3-1
* typo
* update script_server
* auto linking inifiles with ROBOT variable
* corrected base position
* script and parameter for planned motion
* HeadAxis working
* random moves test script
* sound test
* translation has to be in mm
* cob3-1 grasp script modifications
* sound_play node overlay
* Merge branch 'master' of git@github.com:ipa-fmw/care-o-bot
* bugfix
* changes for using planned motion; to be tested on real cob
* scripts using planned motion
* upaload ipa_kitchen params
* end of research-camp
* fix script server
* scrift server fix
* research camp challenge
* research camp challenge
* update folded position
* moved ekf domo publisher to nav; update positions for new urdf trafos; moved controller_manager to cob_controller_configuration_gazebo
* fix for global frame names
* brics exercise 3 working again
* Merge branch 'master' of github.com:ipa-fmw/care-o-bot
* lbr working on cob
* changed trajecotry time back to 3 sec
* Merge branch 'master' of github.com:ipa-fmw/care-o-bot
* new interafaces for kdl solver
* new arm transformation for lbr, set_operation_mode with service interface
* new script table cup, modified time_from_start for all trajectories
* deleted deprecated cob_actions package
* modifications for cob3-1
* allow multiple instances of dashboard
* small modification to script_server
* cleanup in urdfs
* fixes for cob3-1
* added drive by script
* preparation for blocklaser
* head axis working in simulation
* Merged with ipa-320
* Merge completed
* older changes in simple_script_server
* obstacles on floor
* grasp script optimisations
* update urdf to be compatible with ctrutle, add 64bit support for libntcan
* modified script with working cartesian movement
* modified some poses for scripting, changed behaivour of move_cart_rel action
* added joint limit support to ik solver
* testing cart interface
* Merge branch 'master' of github.com:ipa-fmw/care-o-bot
* small fix
* small fixes
* wait for last thred to finish
* bugfix
* added live vizualisation of states
* state information working correctly
* restructured script_server, put more functionality to action handle
* live script_viewer is working
* defined script messages
* script_server levels are working for graph
* publishing is working
* merge with aub, bugfix
* preparing for grasp script
* Merge branch 'scriptserver' of github.com:ipa-fmw/care-o-bot into scriptserver
* merge with aub scriptserver
* graph name is filename
* Merge branch 'scriptserver' of github.com:ipa-fmw/care-o-bot into scriptserver
* graph generation with level
* update positions for lbr
* merge
* performance tuning
* speech suppport for script_server
* update documentation
* source documentation for script server
* added support for live visualization of scripts, lightening up current running procedure has not been done yet
* modified names
* merge
* modified trajectory starttime for better controller performance
* changes to script_server, move_cart_rel still not working
* Merge branch 'review-aub-sss' into scriptserver
* function names changed in script_server
* fixed bug with graph on non string parameters
* merge
* sdh changes and calibration script and parameter
* deps for script_server graph
* working visualization for scripts, needs to be tested, rosrun cob_script_server graph <scriptfile>
* script for camera calibration data
* added graphviz visualization for script server scripts
* modified urdf and adapted xaml files
* Added another file
* Added Mike's Script
* modification for cob3-2
* modi from Reza sample
* temp from reza
* merge and wave files for script server
* Merge branch 'master' of github.com:ipa-uh/care-o-bot
* theo told me to
* bugfix for script_server
* Material for cob_script_server tutorials
* Testing tutorial for cob_script_server
* improved simulation for schunk arm and cleanup in 2dnav package
* fixed init bug
* update dashboard
* update on robot
* dashboard working with script_server
* changed service names to small letters and extended script_server
* grasp from cooler scenarion running
* update script server yaml and lbr urdf description
* update script server yaml and lbr urdf description
* script_server update
* Merge branch 'master' of git@github.com:ipa-fmw/care-o-bot
* First implementation of script to grasp from water cooler
* script server upload files
* Merge branch 'master' of git@github.com:ipa-taj/care-o-bot into review-taj
* small fixes for script_server
* First, untested version of script to grasp from water cooler
* correced files after wrong merge
* Merge branch 'review-fmw'
* Merge branch 'master' of github.com:ipa-taj/care-o-bot
  Conflicts:
  cob_apps/cob_script_server/scripts/test_script.py
  cob_apps/cob_script_server/src/simple_script_server.py
* Sound now tested and working
* update of script_server
* merge with taj
* merge with taj
* rotation around z-axis working, x and y to be changed
* minor modifications to script_server
* Bugfixing on sound section of simple_script_server
* cartesian arm movement is working with script_server
* debuged sound, still not working properly
* merge with taj
* script server working with navigation
* Added sound functionality to simple script server (untested)
* bugfix
* added actionhandler to script_server
* expanded script_server
* moved script_server to open-source repository
* Contributors: Alexander Bubeck, Felix Messmer, Florian Weißhardt, Georg Arbeiter, LucaLattanzi, Mathias Lüdtke, Michael Bowler, Nathan Burke, Tobias Sing, Witalij Siebert, Your full name, abubeck, b-it-bots-secure, cu-noyvirt, fmw, fmw-jiehou, fmw-jk, ipa-fmw, ipa-fxm, ipa-goa, ipa-nhg, ipa-rmb, ipa-taj, ipa-taj-dm, ipa-uhr, ipa-uhr-fm, ipa320, snilsson, uh, uh-mb, uh-reza
