//------------------------------------------------------------------------------
// GRVC MBZIRC 
//------------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2017 GRVC University of Seville
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//------------------------------------------------------------------------------

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <gcs_state_machine.h>
#include <DeployArea.h>

int main(int _argc, char **_argv) {
	ros::init(_argc, _argv, "GCS_STATE_MACHINE");
  
	float deploy_area_radius = 10.0;  // TODO: Test!
	geometry_msgs::Point deploy_area_center;
	deploy_area_center.x = 24;  // TODO: From config file?
	deploy_area_center.y = 30;  // TODO: From config file?
	DeployAreaHandle deploy_area(deploy_area_center, deploy_area_radius);

	GcsStateMachine gcs;
	if(!gcs.init()){
		ROS_ERROR("ERROR INITIALIZING GCS!");
		return -1;	
	}
	
	ros::spin();
}
