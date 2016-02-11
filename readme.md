## Internet access and SSH notes
(GUI instructions only). Click wifi symbol top right, edit connections, Wired connection 1, IPv4 settings. For internet, set Method to Automatic (DHCP). When you want to use this in the car, set method to manual and add an address 192.168.1.4 with netmask 24. No need for a gateway

## ROS setup
Add the end of .bashrc add:

source ~/catkin_ws/devel/setup.bash

export ROS_MASTER_URI=http://192.168.1.2:11311

export ROS_IP=192.168.1.4

## Serial setup
To ensure that the software can access the serial port, run "sudo usermod -a -G dialout pi". 

## Running
Simply use the launch file, ensuring that roscore is running on the main computer.

## Issues
Before running for the first time each boot, the serial port needs to be set up or something. Run "screen /dev/ttyUSB0 38400", ensure a stream of readable data is coming through, then hit ctrl-A and then \.
If you don't do this, then the software will hang on retreiving the first byte.
Also, sometimes the serial port will fail to open at all if you start/stop the program a few times. A reboot is necessary...
