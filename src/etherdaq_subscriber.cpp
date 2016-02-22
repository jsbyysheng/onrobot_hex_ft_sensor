/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, OptoForce, Ltd.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the OptoForce nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#include <stdlib.h>
#include <stdio.h>
#include "std_msgs/Bool.h"
#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"


ros::Time lastTime(0);
ros::Time overallTime(0);
unsigned int packetCount = 0;
ros::Duration accDuration(0);


void chatterCallback(const geometry_msgs::WrenchStamped& msg)
{
	ros::Time currentTime(ros::Time::now());
	
	if (lastTime == ros::Time(0)) {
		lastTime = currentTime;
	}
	if (overallTime.isZero()) {
		overallTime = currentTime;
	} 
	ros::Duration duration = currentTime - lastTime;
	accDuration += duration;
	double durationTime = duration.toSec() * 1000.0;
	double frequency = 0.0;
	++packetCount;
	if (accDuration.toSec() > 1.0) {
		accDuration = ros::Duration(0);
		packetCount = 0;
	}

	if (accDuration.isZero() == false) {
		frequency = (double)packetCount / accDuration.toSec();	
	}



	lastTime = currentTime;
	
	ROS_INFO("%u Fx:%.2f Fy:%.2f Fz:%.2f Tx:%.2f Ty:%.2f Tz:%.2f T:%.2f ms S: %.2f Hz\r\n", msg.header.seq, msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z, durationTime, frequency);

	

	
}


int main (int argc, char ** argv) 
{
	ros::init(argc, argv, "etherdaq_subscriber");
	ros::NodeHandle n;
	ros::Rate loop_rate(1000);  // The loop rate
	ros::Publisher zero_pub = n.advertise<std_msgs::Bool>("ethdaq_zero", 1);  // The topic where we send "zeroing" (Like tare)
	ros::Subscriber sub_raw = n.subscribe("ethdaq_data_raw", 1000, chatterCallback); // The callback where we waiting for Wrench data
	ros::Subscriber sub_new = n.subscribe("ethdaq_data", 1000, chatterCallback);


	ros::Duration zeroingTime(10.0);
	ros::Time lastZeroing = ros::Time::now();
	bool zeroing = true;
	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
		if (lastTime.isZero()) {
			continue;
		}
		ros::Time currentTime = ros::Time::now();
		if (currentTime - lastZeroing >= zeroingTime) {
			// We do a zeroing/unzeroing in every 10 secs
			std_msgs::Bool z;
			z.data = zeroing;
			zero_pub.publish(z);
			zeroing = !zeroing;
			lastZeroing = currentTime;
		}
	}

	return 0;
}
