
###Disable interception of HID devices by bluetoothd
Add the following line in /etc/bluetooth/main.conf

DisablePlugins = input

###Pairing (only once):
sudo bash

rosrun ps3joy sixpair

###Manual Connecting (if no startup script)
sudo bash
rosrun ps3joy ps3joy.py #Controller only

rosrun ps3joy ps3joy_node.py #Controller with Feedback

###Get input (if not in launchfile, without root)
rosrun joy joy_node

###Udev Rules
copy the udev rule 72-ps3.rules (https://raw.githubusercontent.com/ipa320/setup_cob4/master/udev_rules/72-ps3.rules) to /etc/udev/rules.d.

###Autostart script
copy the script ps3joy_node_starter.sh(https://raw.githubusercontent.com/ipa320/setup_cob4/master/udev_rules/ps3joy_node_starter.sh) to /etc/rc.local.


For further information see http://wiki.ros.org/ps3joy
