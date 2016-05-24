/*
 * File for generate waypoints 
 * Author: Luis Manuel Ramirez de la Cova
 * Date: April 2016
 * Organization: University of Seville, GRVC
 * Description: Algorithm 
 */
 
#include <ros/ros.h>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

//Gazebo Libraries
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetPhysicsProperties.h>
#include <gazebo_msgs/ModelState.h>

int main ( int argc, char** argv )
{
    ros::init ( argc, argv, "wp_follower" );
    std::ifstream myReadFile;
    std::string line;
    myReadFile.open("/home/luisgrvc/catkin_ws/src/mbzirc/resource/waypoints.txt");
    if(myReadFile.is_open() == true)
    {
        //char buffer[256];
        float lat, lon, alt, yaw;
        std::cout << "Writing..." << std::endl;
        while (std::getline(myReadFile, line))
        {
            std::string lats = line.substr(0, line.find("\t"));
            line = line.substr(line.find("\t") + 1);
            lat = atof(lats.c_str());
            std::string lons = line.substr(0, line.find("\t"));
            line = line.substr(line.find("\t") + 1);
            lon = atof(lons.c_str());
            std::string alts = line.substr(0, line.find("\t"));
            line = line.substr(line.find("\t") + 1);
            alt = atof(alts.c_str());
            std::string yaws = line.substr(0, line.find("\t"));
            line = line.substr(line.find("\t") + 1);
            yaw = atof(yaws.c_str());
            std::cout << "Receives:" << std::endl << "x:" << lat << " y:" << lon << " z:" << alt << " yaw:" << yaw << std::endl;
        }
        
	ros::NodeHandle n;
	
	ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/SetModelState"); 
        return 0;
    }
    else
    {
        ROS_ERROR("CAN NOT OPEN THE FILE");
        return -1;
    }



}
