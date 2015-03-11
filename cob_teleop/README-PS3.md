See http://wiki.ros.org/ps3joy for Installation and Pairing.
Udev rule should be copied to /etc/udev/rules.d and might require modification for diffrent controller. (Otherwise you have to change the /dev/input/jsPS3 to your controler in the launchfile)
The ps3joy_node_starter.sh from the wiki is modified for re-pairing and belongs in /etc/rc.local (bash /path/to/ps3joy_node_starter.sh) and requires correct HOME and ROS_VERSIO.
The ps3joy_node_starter.sh is available in the ipa320/setup repository

#Pairing (only once):
sudo bash
rosrun ps3joy sixpair

#Manual Connecting (if no startup script)
sudo bash
rosrun ps3joy ps3joy.py #Controller only
rosrun ps3joy ps3joy_node.py #Controller with Feedback

#Get input (if not in launchfile, without root)
rosrun joy joy_node

