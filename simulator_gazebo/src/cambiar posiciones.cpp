/*
 * Change Position of Static Object
 * Author: Luis Manuel Ramirez de la Cova
 * Date: March 2016
 * Organization: University of Seville, GRVC
 * Description: Algorithm that can change randomly the positions
 * of static object for the Testbed MBZIRC
 */
 

#include <ros/ros.h>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "gazebo_msgs/SetModelState.h"
#include <cstdlib>

int main()
{
	ros::init(argc, argv, char **argv);
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
	
	gazebo_msgs::SetModelState setmodelstate;
	gazebo_msgs::ModelState;
	modelstate.model_name = "my_model";
	modelstate.reference_frame = "world";
	
	geometry_msgs::Twist model_twist;
	model_twist.linear.x = 1.0;
	model_twist.linear.y = 1.0;
	model_twist.linear.z = 0.0;
	model_twist.angular.x = 0.25;
	model_twist.angular.y = 0.0;
	model_twist.angular.z = 0.0;
	modelstate.twist = model_twist;
	
	setmodelstate.request.model_state = modelstate;
	
	if(client.call(setmodelstate))
	{
		ROS_INFO("Perfect!!");
		ROS_INFO("%f, %f", modelstate.pose.position.x, modelstate.pose.position.y);
	}
	else
	{
		ROS_ERROR("Failed to call service");
		return 1;
	}
	
	return 0;
}
