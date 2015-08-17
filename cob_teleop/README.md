cob_teleop
====================

General description
---------------------
teleop_node_of_v2

<img src="./model/cob_teleop.png" width="300px" />

Node: cob_teleop_v2
---------------------
#### Parameters
**button_deadman** *(int, default: 11)*
<!--- protected region button_deadman on begin -->
<!--- protected region button_deadman end -->

**base_max_linear** *(double, default: 0.5)*
<!--- protected region base_max_linear on begin -->
<!--- protected region base_max_linear end -->

**base_max_angular** *(double, default: 1.5)*
<!--- protected region base_max_angular on begin -->
<!--- protected region base_max_angular end -->

**torso_max_angular** *(double, default: 0.2)*
<!--- protected region torso_max_angular on begin -->
<!--- protected region torso_max_angular end -->

**head_max_angular** *(double, default: 0.3)*
<!--- protected region head_max_angular on begin -->
<!--- protected region head_max_angular end -->

**sensor_ring_max_angular** *(double, default: 0.1)*
<!--- protected region sensor_ring_max_angular on begin -->
<!--- protected region sensor_ring_max_angular end -->

**arm_joint_velocity_max** *(double, default: 0.3)*
<!--- protected region arm_joint_velocity_max on begin -->
<!--- protected region arm_joint_velocity_max end -->

**arm_cartesian_max_linear** *(double, default: 0.1)*
<!--- protected region arm_cartesian_max_linear on begin -->
<!--- protected region arm_cartesian_max_linear end -->

**arm_cartesian_max_angular** *(double, default: 0.1)*
<!--- protected region arm_cartesian_max_angular on begin -->
<!--- protected region arm_cartesian_max_angular end -->

**gripper_max_velocity** *(double, default: 0.1)*
<!--- protected region gripper_max_velocity on begin -->
<!--- protected region gripper_max_velocity end -->

**base_x** *(int, default: 1)*
<!--- protected region base_x on begin -->
<!--- protected region base_x end -->

**base_y** *(int, default: 0)*
<!--- protected region base_y on begin -->
<!--- protected region base_y end -->

**base_yaw** *(int, default: 2)*
<!--- protected region base_yaw on begin -->
<!--- protected region base_yaw end -->

**arm_x** *(int, default: 0)*
<!--- protected region arm_x on begin -->
<!--- protected region arm_x end -->

**arm_y** *(int, default: 1)*
<!--- protected region arm_y on begin -->
<!--- protected region arm_y end -->

**arm_yaw** *(int, default: 2)*
<!--- protected region arm_yaw on begin -->
<!--- protected region arm_yaw end -->

**arm_pitch_up** *(int, default: 4)*
<!--- protected region arm_pitch_up on begin -->
<!--- protected region arm_pitch_up end -->

**arm_pitch_down** *(int, default: 6)*
<!--- protected region arm_pitch_down on begin -->
<!--- protected region arm_pitch_down end -->

**arm_roll_right_and_ellbow** *(int, default: 5)*
<!--- protected region arm_roll_right_and_ellbow on begin -->
<!--- protected region arm_roll_right_and_ellbow end -->

**arm_roll_left_and_ellbow** *(int, default: 7)*
<!--- protected region arm_roll_left_and_ellbow on begin -->
<!--- protected region arm_roll_left_and_ellbow end -->

**arm_z_up** *(int, default: 12)*
<!--- protected region arm_z_up on begin -->
<!--- protected region arm_z_up end -->

**arm_z_down** *(int, default: 14)*
<!--- protected region arm_z_down on begin -->
<!--- protected region arm_z_down end -->

**gripper_open** *(int, default: 15)*
<!--- protected region gripper_open on begin -->
<!--- protected region gripper_open end -->

**gripper_close** *(int, default: 13)*
<!--- protected region gripper_close on begin -->
<!--- protected region gripper_close end -->

**arm_joint_up** *(int, default: 4)*
<!--- protected region arm_joint_up on begin -->
<!--- protected region arm_joint_up end -->

**arm_joint_down** *(int, default: 6)*
<!--- protected region arm_joint_down on begin -->
<!--- protected region arm_joint_down end -->

**arm_joint_left** *(int, default: 7)*
<!--- protected region arm_joint_left on begin -->
<!--- protected region arm_joint_left end -->

**arm_joint_right** *(int, default: 5)*
<!--- protected region arm_joint_right on begin -->
<!--- protected region arm_joint_right end -->

**arm_joint_12** *(int, default: 15)*
<!--- protected region arm_joint_12 on begin -->
<!--- protected region arm_joint_12 end -->

**arm_joint_34** *(int, default: 14)*
<!--- protected region arm_joint_34 on begin -->
<!--- protected region arm_joint_34 end -->

**arm_joint_56** *(int, default: 13)*
<!--- protected region arm_joint_56 on begin -->
<!--- protected region arm_joint_56 end -->

**arm_joint_7_gripper** *(int, default: 12)*
<!--- protected region arm_joint_7_gripper on begin -->
<!--- protected region arm_joint_7_gripper end -->

**axis_runfactor** *(int, default: 9)*
<!--- protected region axis_runfactor on begin -->
<!--- protected region axis_runfactor end -->

**button_safety_override** *(int, default: 9)*
<!--- protected region button_safety_override on begin -->
<!--- protected region button_safety_override end -->

**button_init_recover** *(int, default: 3)*
<!--- protected region button_init_recover on begin -->
<!--- protected region button_init_recover end -->

**button_mode_switch** *(int, default: 0)*
<!--- protected region button_mode_switch on begin -->
<!--- protected region button_mode_switch end -->

**torso_roll** *(int, default: 0)*
<!--- protected region torso_roll on begin -->
<!--- protected region torso_roll end -->

**torso_pitch** *(int, default: 1)*
<!--- protected region torso_pitch on begin -->
<!--- protected region torso_pitch end -->

**torso_yaw_left** *(int, default: 15)*
<!--- protected region torso_yaw_left on begin -->
<!--- protected region torso_yaw_left end -->

**torso_yaw_right** *(int, default: 13)*
<!--- protected region torso_yaw_right on begin -->
<!--- protected region torso_yaw_right end -->

**sensorring_yaw_left** *(int, default: 4)*
<!--- protected region sensorring_yaw_left on begin -->
<!--- protected region sensorring_yaw_left end -->

**sensorring_yaw_right** *(int, default: 6)*
<!--- protected region sensorring_yaw_right on begin -->
<!--- protected region sensorring_yaw_right end -->

**head_roll** *(int, default: 2)*
<!--- protected region head_roll on begin -->
<!--- protected region head_roll end -->

**head_pitch** *(int, default: 3)*
<!--- protected region head_pitch on begin -->
<!--- protected region head_pitch end -->

**head_yaw_left** *(int, default: 7)*
<!--- protected region head_yaw_left on begin -->
<!--- protected region head_yaw_left end -->

**head_yaw_right** *(int, default: 5)*
<!--- protected region head_yaw_right on begin -->
<!--- protected region head_yaw_right end -->

**head_home** *(int, default: 4)*
<!--- protected region head_home on begin -->
<!--- protected region head_home end -->

**arm_left_home** *(int, default: 7)*
<!--- protected region arm_left_home on begin -->
<!--- protected region arm_left_home end -->

**arm_right_home** *(int, default: 5)*
<!--- protected region arm_right_home on begin -->
<!--- protected region arm_right_home end -->

**torso_home** *(int, default: 6)*
<!--- protected region torso_home on begin -->
<!--- protected region torso_home end -->

**sensorring_home** *(int, default: 12)*
<!--- protected region sensorring_home on begin -->
<!--- protected region sensorring_home end -->

**gripper_left_home** *(int, default: 15)*
<!--- protected region gripper_left_home on begin -->
<!--- protected region gripper_left_home end -->

**gripper_right_home** *(int, default: 13)*
<!--- protected region gripper_right_home on begin -->
<!--- protected region gripper_right_home end -->

**base_home** *(int, default: 14)*
<!--- protected region base_home on begin -->
<!--- protected region base_home end -->

**arm_left_uri** *(XmlRpcValue, default: )*
<!--- protected region arm_left_uri on begin -->
<!--- protected region arm_left_uri end -->

**components** *(XmlRpcValue, default: )*
<!--- protected region components on begin -->
<!--- protected region components end -->

**home_time** *(double, default: 5.0)*
<!--- protected region home_time on begin -->
<!--- protected region home_time end -->

**stop_time** *(double, default: 0.8)*
<!--- protected region stop_time on begin -->
<!--- protected region stop_time end -->

**arm_right_uri** *(XmlRpcValue, default: )*
<!--- protected region arm_right_uri on begin -->
<!--- protected region arm_right_uri end -->

**led_mode** *(XmlRpcValue, default: )*
<!--- protected region led_mode on begin -->
<!--- protected region led_mode end -->

**gripper_1** *(int, default: 3)*
<!--- protected region gripper_1 on begin -->
<!--- protected region gripper_1 end -->

**gripper_2** *(int, default: 2)*
<!--- protected region gripper_2 on begin -->
<!--- protected region gripper_2 end -->

**gripper_max_angular** *(double, default: 0.2)*
<!--- protected region gripper_max_angular on begin -->
<!--- protected region gripper_max_angular end -->


#### Published Topics
**joy_feedback** *(sensor_msgs::JoyFeedbackArray)*
<!--- protected region joy_feedback on begin -->
<!--- protected region joy_feedback end -->

**base_controller command** *(geometry_msgs::Twist)*
<!--- protected region base_controller command on begin -->
<!--- protected region base_controller command end -->

**arm_cart_left** *(geometry_msgs::Twist)*
<!--- protected region arm_cart_left on begin -->
<!--- protected region arm_cart_left end -->

**arm_cart_right** *(geometry_msgs::Twist)*
<!--- protected region arm_cart_right on begin -->
<!--- protected region arm_cart_right end -->

**arm_joint_right** *(std_msgs/Float64MultiArray)*
<!--- protected region arm_joint_right on begin -->
<!--- protected region arm_joint_right end -->

**arm_joint_left** *(std_msgs/Float64MultiArray)*
<!--- protected region arm_joint_left on begin -->
<!--- protected region arm_joint_left end -->

**head_controller command** *(geometry_msgs::Twist)*
<!--- protected region head_controller command on begin -->
<!--- protected region head_controller command end -->

**sensorring_controller command** *(std_msgs::Float64MultiArray)*
<!--- protected region sensorring_controller command on begin -->
<!--- protected region sensorring_controller command end -->

**torso_controller command** *(geometry_msgs::Twist)*
<!--- protected region torso_controller command on begin -->
<!--- protected region torso_controller command end -->

**gripper_left** *(std_msgs::Float64MultiArray)*
<!--- protected region gripper_left on begin -->
<!--- protected region gripper_left end -->

**gripper_right** *(std_msgs::Float64MultiArray)*
<!--- protected region gripper_right on begin -->
<!--- protected region gripper_right end -->


#### Subscribed Topics
**joy** *(sensor_msgs::Joy)*
<!--- protected region joy on begin -->
<!--- protected region joy end -->



