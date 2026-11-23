## Hardware Requirements

This package uses a joystick or keyboard for teleoperation of
Care-O-bot. To use this package you need either a real Care-O-bot or a
simulated one (see [cob_gazebo](https://index.ros.org/p/cob_gazebo)).

## ROS API

The cob_teleop package provides both, a
configurable node for teleoperation by joystick or by keyboard.

### cob_teleop

The `cob_teleop` node takes in
[sensor_msgs/msg/Joy](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Joy.html)
messages and send direct commands to the hardware.

#### Subscribed Topics

- `joy`
  ([sensor_msgs/msg/Joy](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Joy.html)):
  Receives joystick commands.
- `/joint_states`
  ([sensor_msgs/msg/JointState](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/JointState.html)):
  Receives the joint states of all joints for initialization.

#### Published Topics

- `arm_controller/command`
  ([trajectory_msgs/msg/JointTrajectory](https://docs.ros.org/en/rolling/p/trajectory_msgs/msg/JointTrajectory.html)):
  Publishes commands to the arm.
- `torso_controller/command`
  ([trajectory_msgs/msg/JointTrajectory](https://docs.ros.org/en/rolling/p/trajectory_msgs/msg/JointTrajectory.html)):
  Publishes commands to the torso.
- `tray_controller/command`
  ([trajectory_msgs/msg/JointTrajectory](https://docs.ros.org/en/rolling/p/trajectory_msgs/msg/JointTrajectory.html)):
  Publishes commands to the tray.
- `base_controller/command`
  ([geometry_msgs/msg/Twist](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/Twist.html)):
  Publishes commands to the base.

#### Parameters

- `~run_factor` (`float`, default: `1.5`): Defines the factor to
  multiply velocities when run button is pressed.
- `~lower_neck_button` (`int`, default: `6`): Joystick button for lower
  neck.
- `~upper_neck_button` (`int`, default: `4`): Joystick button for upper
  neck.
- `~tray_button` (`int`, default: `3`): Joystick button for tray.
- `~arm_joint12_button` (`int`, default: `0`): Joystick button for joint
  1 and 2 of arm.
- `~arm_joint34_button` (`int`, default: `1`): Joystick button for joint
  3 and 4 of arm.
- `~arm_joint56_button` (`int`, default: `2`): Joystick button for joint
  5 and 6 of arm.
- `~arm_joint7_button` (`int`, default: `3`): Joystick button for joint
  7 of arm.
- `~deadman_button` (`int`, default: `5`): Joystick button for deadman
  functionality.
- `~run_button` (`int`, default: `7`): Joystick button run
  functionality.
- `~axis_vx` (`int`, default: `1`): Joystick axis for base movements
  along x-axis.
- `~axis_vy` (`int`, default: `0`): Joystick axis for base movements
  along y-axis.
- `~axis_vth` (`int`, default: `2`): Joystick axis for base movements
  around tetha-axis.
- `~lower_tilt_step` (`float`, default: `0.05`): Step for lower tilt
  movements in rad/sec.
- `~lower_pan_step` (`float`, default: `0.05`): Step for lower pan
  movements in rad/sec.
- `~upper_tilt_step` (`float`, default: `0.075`): Step for upper tilt
  movements in rad/sec.
- `~upper_pan_step` (`float`, default: `0.075`): Step for upper pan
  movements in rad/sec.
- `~tray_step` (`float`, default: `0.1`): Step for tray movements in
  rad/sec.
- `~arm_left_right_step` (`float`, default: `0.1`): Step for left and
  right arm movements in rad/sec.
- `~arm_up_down_step` (`float`, default: `0.1`): Step for up and down
  arm movements in rad/sec.
- `~max_vx` (`float`, default: `0.3`): Maximum velocity for base
  movements along x-axis in m/sec.
- `~max_vy` (`float`, default: `0.2`): Maximum velocity for base
  movements along y-axis in m/sec.
- `~max_vth` (`float`, default: `0.3`): Maximum velocity for base
  movements around tetha-axis in rad/sec.
- `~max_ax` (`float`, default: `0.5`): Maximum acceleration for base
  movements along x-axis in m/sec\^2.
- `~max_ay` (`float`, default: `0.5`): Maximum acceleration for base
  movements along y-axis in m/sec\^2.
- `~max_ath` (`float`, default: `0.5`): Maximum acceleration for base
  movements around tetha-axis in rad/sec\^2.

## Usage/Examples

### Starting teleoperation node

For starting the teleoperation with joystick 

```
roslaunch cob_bringup teleop.launch
```


For including the teleoperation in your overall launch file use


```
<include file="$(find cob_bringup)/tools/teleop.launch" />
```

 If you haven't got joystick you can move the
base of the robot using the keyboard: 

```
roslaunch cob_teleop teleop_keyboard.launch
```



### Move components

To be able to use the joystick the `deadman_button` has to be pressed
all the time, as soon as the button is released a stop will be send to
all hardware components. Have a look at the following image to see which
buttons command which components. ![width=100%](doc/teleop.png)

For **moving the base**: Hold the `deadman` button and use the base
`rotation` and `translation` axis to move the base.

For **moving the torso**: Hold the `deadman` button and the `upper`
or `lower` neck button, then use the `up_down` or `left_right`
axis to move the torso.

For **moving the tray**: Hold the `deadman` button and the `tray`
button, then use the `up_down` axis to move the tray.

For **moving the arm**: Hold the `deadman` button and one of the
`arm` buttons, then use the `up_down` or `left_right` axis to move
the selected arm joints.

### Parameter file

A sample parameter file could look like this 

```
# common params
run_factor: 2.0

# buttons
lower_neck_button: 6
upper_neck_button: 4
tray_button: 3
arm_joint12_button: 0
arm_joint34_button: 1
arm_joint56_button: 2
arm_joint7_button: 3
deadman_button: 5
run_button: 7

# axes
axis_vx: 1
axis_vy: 0
axis_vth: 2
up_down: 5 #tray--up/down; tilt--front/back, here we just name up_down
left_right: 4 #pan--left/right

# velocities in rad/sec, m/sec
lower_tilt_step: 0.05
lower_pan_step: 0.05
upper_tilt_step: 0.075
upper_pan_step: 0.075
tray_step: 0.15
arm_left_right_step: 0.1
arm_up_down_step: 0.1
max_vx: 0.3
max_ax: 0.5
max_vy: 0.2
max_ay: 0.5
max_vth: 0.3
max_ath: 0.7
```
