^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_teleop
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Revert "catkin_lint'ing"
  This reverts commit f7fb5ce0e5d76bc44795beee9a40e9a87a863bc6.
* catkin_lint'ing
* Delete .project
* Delete .cproject
* Merge branch 'hydro_dev' of github.com:ipa-fxm/cob_command_tools into indigo_dev
* The license tag must neither be empty
* add layout as pdf and gimp
* added README with instructions
* little cleanup
* init recover all in automoves
* gripper support for 2 joints
* service call with function
* move from sss to service calls
* led modes with yaml
* create symlink for input device
* fixed udev rules but still unreliable
* regenerated with newest bride version
* LED feedback
* make joaystick available after roscore restart
* Cleanup for indigo and rewrite of dashboard to run without pr2_msgs
* starting with ps3 status LEDs
* removed git specific file
* picked into old branch
* removed .metadata again, wasn't in local gitignore
* check if component stopped
* merged other repo
* stop components once via sss after deadman release
* merged yaml change
* changed arm twist topic
* some testing
* created launch file
* attempting usage of brides new XmlRpc
* actionlib working
* adapted to new bride version
* init_recover only once per press
* allowing homing only once per press. Restored protected region
* attemp init recover for components
* actionlib moves working except for scope visibility
* attempt action client for script server
* added right arm
* arm joint control working
* tried arm joints, but get size mismatch from vel_controller
* forgot arm_joint_velocity_max
* implemented torso. working
* some simplifying
* removed .metadata
* added comments, fixed runf_factor
* first time publishing bricsActuator/JointVelocities
* moved folder
* Contributors: Alexander Bubeck, Felix Messmer, Florian Weisshardt, ipa-cob4-2, ipa-fmw-ms, ipa-fxm

0.5.1 (2014-03-20)
------------------
* fixed cob_teleop_keyboard
* fix teleop for 3DOF torso
* changes for hydro deps
* Fixed CMakefiles for teleop stuff.
* merged catkin version
* Initial catkinization.
* critial bugfix (buffer overflow)
* fixed wrong debug message
* use 100Hz for teleop
* no waiting for parameters
* fuerte migration, joy msg moved
* adapt roslaunch tests
* fix safety
* teleop with safe base movements
* add dependency to joy
* removed deprecated dependency
* new file teleop_keyboard.launch
* fix robot modules
* removed launch files
* removed launch and configuration files
* remove compiler warnings
* use joy.launch in teleop
* removed compiler warnings
* added cob3-4 configs
* removed compiler warninigs
* config files for cob3-bosch
* added ENV variables to tests
* electric update for teleop
* cleanup arm and dashboard configs
* add stop and recover/init button to teleop
* config for cob3-3
* Merge branch 'master' of github.com:ipa-fmw/cob_apps
* added license header
* add recover base button to teleop
* cob3-2 config for teleop
* changed tinmeout to 1sec
* merge
* teleop with brics messages
* added desire.yaml
* merge
* moved output to DEBUG
* added some usage instruction output
* small bug-fix
* new teleop_keyboard version - includes arm, tray, torso
* fixed teleop jump-back error
* deleted old launch file
* added module parameters for all modules
* added yaml teleop module yaml file for cob3-1
* moved robot specific teleop configuration to external configuration files
* merge
* removed deprecated dependencies
* add dependency to pt2_teleop
* wait only for 1 sec
* added support for brics intefaces to tray and arm
* added brics interface for torso
* cleanup in cob_apps and updated stack.xml's
* research camp challenge
* merge
* renamed camera_axis to head_axis and platform to base
* system cleaned - missing launch files added
* much ado about nothing
* Modified launch files of cob_base_drive_chain, cob_relayboard, cob_undercaariage_ctrl and cob_teleop_ucar and made them hierarchic
* merged with cpc-pk: added ctrl for tricycle-kinematic; specification of limit in CanDriveHarmonica can now be specified via Inifile; base_drive_chain can be operated on variable numbers of motors (lesser or equal to eight); variable setting of path to inifile for UndercarriageCtrlGeom; debugged relaysboard - reads Bus now nonblocking
* -
* merge
* teleop keyboard
* Merge branch 'master' into scriptserver
* performance tuning
* teleoperation with keyboard
* update documentation
* bugfix in teleop
* ramp filter for base_controller
* teleop with deadman and run button
* improved joystick handling
* renamed launch file
* modification on cob3-2
* adaptions for cob3-2
* knoeppkes
* new platform launch file
* deleted teleop keyboard
* update on robot
* dual arm cob3 simulation and modified controllers for schunk simulation
* modifications for navigation with ucar
* adapt device
* merge with cpc
* Added dependencies for build of controllers to cob_teleop package
* implemented, debugged and tested basic undercarriage controller - works on Descartes principal of rigid body motion
* remote controll of torso, tray, arm with joystick is working
* added timeout, if no /joint_states message arrives
* initial values for velocities
* get initial joint values from joint_states topic
* test
* Deployment of undercarriage controller debugged and finished: launch-script cob_ucar_joy starts up relayboard, base_drive_chain and controller; also remaps topics and services in correct namespaces. Debugging of controller itself is work in progress: simplified and removed old stuff - code compiles - controller runs but appaerently has some bugs -> may not yet be used
* Merge branch 'review-cpc'
* introduced env variable ROBOT
* debugging undercarriage drivers (base_drive_chain + relayboard + ucar_ctrl) - work in progress
* cleaning up in cob_apps stack
* modified teleop launch file
* launch file for teleop_cob
* new teleop for cob
* merge
* new stl files for torso
* JSF: Added intrinsics to topic
* debugged ucar controller and base drive chain node - still not running
* reduced velocity of joystick
* better 2d navigation
* test of ROS navigation on cob
* renamed packages to cob_ convention
* Contributors: Alexander Bubeck, COB3-Manipulation, COB3-Navigation, Christian, Florian Weißhardt, Your full name, abubeck, b-it-bots, cpc, fmw, ipa, ipa-bnm, ipa-cob3-3, ipa-cpc, ipa-fmw, ipa-fxm, ipa-taj-dm, ipa-uhr-fm, nhg-ipa, snilsson, uh
