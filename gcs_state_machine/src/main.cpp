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
#include <gcs_state_machine/gcs_state_machine.h>
#include <gcs_state_machine/deploy_area_handle.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int _argc, char **_argv) {
	if (_argc < 2) {
		std::cout << "Not enough input arguments! Please provide any UAV id." << std::endl;
		return -1;
	}

	ros::init(_argc, _argv, "gcs_state_machine");

	float deploy_area_radius = 8.0;  // TODO: Test!

	tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
	geometry_msgs::TransformStamped transformToMap;
	geometry_msgs::PointStamped game_deploy_point;
	geometry_msgs::PointStamped deploy_point;

	// Extract deploy point from params
	std::vector<double> dropping_limits;

	ros::param::get("/dropping_limits/limits", dropping_limits);

	double deploy_x = (dropping_limits[0] + dropping_limits[1])/2.0;
	double deploy_y = (dropping_limits[2] + dropping_limits[3])/2.0;

	game_deploy_point.header.frame_id = "game";
	game_deploy_point.point.x = deploy_x;
	game_deploy_point.point.y = deploy_y;
	game_deploy_point.point.z = 3.0;

	transformToMap = tfBuffer.lookupTransform("map", "game", ros::Time(0), ros::Duration(0.2));
	tf2::doTransform(game_deploy_point, deploy_point, transformToMap);

	DeployAreaHandle deploy_area(deploy_point.point, deploy_area_radius);

	// Assumming that _argv has the path of the executable and the ids of the uavs to be launch.
	std::vector<int> uavs_id;
	for (int i = 1; i < _argc; i++) {
		uavs_id.push_back(atoi(_argv[i]));
	}

	GcsStateMachine gcs;
	if (!gcs.init(uavs_id)) {
		ROS_ERROR("Error initializing gcs!");
		return -1;	
	}

	ros::spin();
}
