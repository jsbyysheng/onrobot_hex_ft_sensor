# etherdaq_ros
The driver has been tested under Ubuntu 14.04 and ROS indigo.
To build the sources you should use catkin


Compiling
---------
Assuming you are in home directory, and you have root priviliges first you should do the following steps

In a console, type:

 * mkdir opto_ws
 * cd opto_ws
 * mkdir src
 * cd src
 * catkin_init_workspace
 * git clone https://github.com/OptoForce/etherdaq_ros.git
 * cd ..
 * catkin_make

After successful compiling, you will have two executable files and one library file.


Usage - Demo
------------

First, should check if the EtherDAQ is working by OptoForce Ethernet Discovery Tool. The tool will 
show you the address of your EtherDAQ device(s) connected to your network.

To be able to run the ROS demo, you have to modify the following file: 
opto_ws/src/opto_ros_driver/ethernet_daq_driver/launch/demo.launch
Replace 192.168.100.12 in the following line args="--address 192.168.100.12 --rate 1000 --filter 4"
to your EtherDAQ's address provided by the Ethernet Discovery Tool.

To run the demo be sure that you are in the home/opto_ws directory!
Type roslaunch optoforce_etherdaq_driver demo.launch

You should see something similar on your console:


[ INFO] [1455791190.094318011]: 2559 Fx:-116.00 Fy:122.00 Fz:141.00 Tx:-183.00 Ty:66.00 Tz:98.00 T:0.06 ms S: 884.97 Hz

[ INFO] [1455791190.095304760]: 2560 Fx:-116.00 Fy:122.00 Fz:141.00 Tx:-183.00 Ty:66.00 Tz:99.00 T:0.98 ms S: 885.17 Hz

[ INFO] [1455791190.096267234]: 2561 Fx:-116.00 Fy:122.00 Fz:141.00 Tx:-184.00 Ty:65.00 Tz:98.00 T:0.96 ms S: 885.40 Hz

[ INFO] [1455791190.097198648]: 2562 Fx:-116.00 Fy:121.00 Fz:141.00 Tx:-183.00 Ty:66.00 Tz:99.00 T:0.93 ms S: 885.67 Hz

[ INFO] [1455791190.097247749]: 2563 Fx:-117.00 Fy:121.00 Fz:141.00 Tx:-183.00 Ty:66.00 Tz:99.00 T:0.05 ms S: 887.15 Hz

[ INFO] [1455791190.099165675]: 2564 Fx:-117.00 Fy:121.00 Fz:141.00 Tx:-184.00 Ty:67.00 Tz:99.00 T:1.91 ms S: 886.08 Hz

[ INFO] [1455791190.099251998]: 2565 Fx:-117.00 Fy:121.00 Fz:141.00 Tx:-184.00 Ty:67.00 Tz:99.00 T:0.09 ms S: 887.50 Hz


If the EtherDAQ does not repsonse, please double check the address in the demo.launch file.




Usage - Real world
------------------


The ROS driver is a simple node providing Force/Torque informations on a ROS topic.
After compiling you can find the node in the /opto_ws/devel/lib/optoforce_etherdaq_driver.



Parameters of the node
----------------------
* --help Shows the help screen and exits.
* --rate (default: 100) (in Hz) The publish speed of the F/T informations. It also sets the EtherDAQ speed to the given value. 
* --filter (default: 4) Set the filtering (Valid parameters: 0 = No filter; 1 = 500 Hz; 2 = 150 Hz; 3 = 50 Hz; 4 = 15 Hz; 5 = 5 Hz; 6 = 1.5 Hz)
* --address The IP address of the EtherDAQ device.
* --wrench  publish older Wrench message type instead of WrenchStamped
* --frame_id arg (default: "base_link") Frame ID for Wrench data


Example: 
 "./etherdaq_node --address 192.168.100.12 --rate 500 --filter 3"
* This will start the node with 500 Hz publish rate and with 50 Hz filter.


Topics of the node
------------------
The node subscribes to /etdaq_zero where you can zero the force/torque readings at the current loading level.
 * The parameter is std_msgs::Bool, if it's true, then the node zeroes otherwise it unzeroes.

The node publishes to the following topics:
*   /diagnostics where you can check the status of the EtherDAQ (speed, last F/T values, system status, address, etc)
*   /etherdaq_data the topic where F/T values are published either in Wrench or in WrenchStamped format if the force and torque units are given 
*   /etherdaq_data_raw topic where F/T values are published either in Wrench or in WrenchStamped format if the force and torque units are not given



The list and short description of source files
----------------------------------------------

* etherdaq_driver.cpp: 	   This is the module which implements the communication between EtherDAQ and ROS.
* etherdaq_node.cpp: 	   This is the module which publish F/T values on ROS topics using etherdaq_driver.cpp 
                           services.
* etherdaq_subscriber.cpp: An example node which subscribes to the topics of etherdaq_node and displays
                           the actual F/T, speed and elapsed times between two packets. Also this node 
                           does a zero/unzero in every 10 seconds.
