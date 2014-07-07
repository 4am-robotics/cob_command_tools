See http://wiki.ros.org/ps3joy for Installation and Pairing.
Udev rule should be copied to /etc/udev and might require modification for diffrent controller.
Otherwise you have to change the /dev/input/jsPS3 to your controler in the launchfile
The ps3joy_node_starter.sh from the wiki is modified for re-pairing and belongs in /etc/rc.local and REQUIRES Path modification for your system.

