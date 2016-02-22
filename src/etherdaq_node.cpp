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

/** 
 * Simple stand-alone ROS node that takes data from EtherDAQ sensor and
 * Publishes it ROS topic
 */

#include "ros/ros.h"
#include "etherdaq_driver/etherdaq_driver.h"
#include "geometry_msgs/WrenchStamped.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "std_msgs/Bool.h"
#include <unistd.h>
#include <iostream>
#include <memory>
#include <boost/program_options.hpp>

namespace po = boost::program_options;
using namespace std;



optoforce_etherdaq_driver::EtherDAQDriver * etherdaq = NULL;

void zeroFunction(const std_msgs::Bool &msg)
{
	if (etherdaq == NULL) {
		return;
	}
	bool zeroing = msg.data;
	if (zeroing) {
		etherdaq->doZero();
		return;
	}
	etherdaq->doUnzero();
}


int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "etherdaq_node");
  ros::NodeHandle nh;

  float pub_rate_hz;
  int filter_hz; 
  string address;
  string frame_id;

  po::options_description desc("Options");
  desc.add_options()
    ("help", "display help")
    ("rate", po::value<float>(&pub_rate_hz)->default_value(100.0), "set publish rate and Ethernet DAQ speed (in hertz)")
	("filter", po::value<int>(&filter_hz)->default_value(4), "set filtering (0 = No filter; 1 = 500 Hz; 2 = 150 Hz; 3 = 50 Hz; 4 = 15 Hz; 5 = 5 Hz; 6 = 1.5 Hz)") 
    ("wrench", "publish older Wrench message type instead of WrenchStamped")
    ("address", po::value<string>(&address), "IP address of EthernetDAQ box")
    ("frame_id", po::value<string>(&frame_id)->default_value("base_link"), "Frame ID for Wrench data")  
    ;
     
  po::positional_options_description p;
  p.add("address",  1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    cout << desc << endl;
    exit(EXIT_SUCCESS);
  }      

  if (!vm.count("address"))
  {
    cout << desc << endl;
    cerr << "Please specify address of EthernetDAQ" << endl;
    exit(EXIT_FAILURE);
  }
	
  if (filter_hz < 0 || filter_hz > 6) {
	  cout << desc << endl;
	  cerr<<"Please specify a valid filtering value instead of "<<filter_hz<<endl;
	  exit(EXIT_FAILURE);
  }

  bool publish_wrench = false;
  if (vm.count("wrench"))
  {
    publish_wrench = true;
    ROS_WARN("Publishing EthernetDAQ data as geometry_msgs::Wrench is deprecated");
  }

  etherdaq = new optoforce_etherdaq_driver::EtherDAQDriver(address, pub_rate_hz, filter_hz);
	
  bool isRawData = etherdaq->isRawData();

  std::string topicName = "ethdaq_data";	
  if (isRawData) {
	  topicName += "_raw";
  }
	
  ros::Publisher pub;
  ros::Subscriber sub = nh.subscribe("ethdaq_zero", 1000, zeroFunction);
  if (publish_wrench)
  {
    pub = nh.advertise<geometry_msgs::Wrench>(topicName, 100);
  }
  else 
  {
    pub = nh.advertise<geometry_msgs::WrenchStamped>(topicName, 100);
  }
  ros::Rate pub_rate(pub_rate_hz);
  geometry_msgs::WrenchStamped data;

  ros::Duration diag_pub_duration(1.0);
  ros::Publisher diag_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 2);
  diagnostic_msgs::DiagnosticArray diag_array;
  diag_array.status.reserve(1);
  diagnostic_updater::DiagnosticStatusWrapper diag_status;
  ros::Time last_diag_pub_time(ros::Time::now());

  unsigned int packetCount = 0; 
  ros::Time startTime(ros::Time::now());	
  while (ros::ok())
  {
    if (etherdaq->waitForNewData())
    {
      etherdaq->getData(data);
      packetCount++; 
      if (publish_wrench) 
      {
	data.header.frame_id = frame_id;
        pub.publish(data.wrench);
      }
      else 
      {
   	data.header.frame_id = frame_id;
        pub.publish(data);
      }
    }
    
    ros::Time current_time(ros::Time::now());
    if ( (current_time - last_diag_pub_time) > diag_pub_duration )
    {
      diag_array.status.clear();
      etherdaq->diagnostics(diag_status);
      diag_array.status.push_back(diag_status);
      diag_array.header.stamp = ros::Time::now();
      diag_pub.publish(diag_array);
      last_diag_pub_time = current_time;
    }

    ros::spinOnce();
    pub_rate.sleep();
  }
	
  if (etherdaq != NULL) {
	  delete etherdaq;
  }
  
  return 0;
}
