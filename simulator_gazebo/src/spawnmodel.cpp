#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include <iostream>
#include <fstream>
#include <std_srvs/Empty.h>
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Quaternion.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/BodyRequest.h"
#include "gazebo_msgs/DeleteModel.h"


//ros::ServiceClient client_spawn;
gazebo_msgs::SpawnModel uav1;

ros::ServiceClient client_delete_uav1;
gazebo_msgs::DeleteModel delete_uav1;

geometry_msgs::Point pos_obj;
float orientation;
std::string type_objetive;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "spawn_model");
	ros::NodeHandle n;
	
	ros::ServiceClient clientspawn = n.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/SpawnModel");
	

	
	ros::spin();
	
	return 0;
}
