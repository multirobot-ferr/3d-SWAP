/*
 * File for change positions of UAVs
 * Author: Luis Manuel Ramirez de la Cova
 * Created: April 2016
 * Modificate: 05-05-2016
 * Organization: University of Seville, Group Robotics, Vision and Control (GRVC)
 * Description: Algorithm that can to change the positions of each UAV
 */
 
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include "ros/ros.h"
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/GetModelState.h" 
#include "gazebo_msgs/GetPhysicsProperties.h"
#include "gazebo_msgs/ModelState.h"
#include <geometry_msgs/Twist.h>

  int main(int argc, char** argv)
   {

    ros::init(argc, argv, "pruebamover");
    
    std::ifstream myReadFile;
    std::string line;
    myReadFile.open("/home/luisgrvc/catkin_ws/src/mbzirc/resource/waypoints.txt");
    
    sleep(20);

    float x, y, z;
    

    ros::NodeHandle n;
  //  ros::NodeHandle nh;

    //ros::Subscriber sub = n.subscribe("/gazebo/set_model_state", 1000);

    ros::ServiceClient gpp_c = n.serviceClient<gazebo_msgs::GetPhysicsProperties>("/gazebo/get_physics_properties", 100);
    gazebo_msgs::GetPhysicsProperties getphysicsproperties;
    gpp_c.call(getphysicsproperties);

    double t=0.1;

    t=getphysicsproperties.response.time_step + t;

    ros::ServiceClient gms_c = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    gazebo_msgs::GetModelState getmodelstate;
    gms_c.call(getmodelstate);

    ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    gazebo_msgs::SetModelState setmodelstate;

    geometry_msgs::Twist twist;
    geometry_msgs::Pose pose;

    if(myReadFile.is_open()==true)
    {
    	while (std::getline(myReadFile, line))
        {

            twist.linear.x = 2;
            //twist.linear.y = 2;
            //twist.linear.z = 2;
           // twist.angular.x = 0.002;
            //twist.angular.y = 0.002;
            twist.angular.z = 2;
      //      pub.publish(twist);

            gazebo_msgs::ModelState modelstate;
            modelstate.model_name ="Object_Black_Cylinder1";

            std::string xPoss = line.substr(0, line.find("\t"));
            line = line.substr(line.find("\t") + 1);
            x = atof(xPoss.c_str());

            //geometry_msgs::Pose pose;
            pose.position.x = getmodelstate.response.pose.position.x + getphysicsproperties.response.time_step*twist.linear.x + x;
            pose.position.y = getmodelstate.response.pose.position.y + getphysicsproperties.response.time_step*twist.linear.y + y;
            pose.position.z = getmodelstate.response.pose.position.z + getphysicsproperties.response.time_step*twist.linear.z + z;
            pose.orientation.x = 0.0;
            pose.orientation.y = 0.0;
            pose.orientation.z = 0.0;

            modelstate.pose = pose;
            modelstate.twist = twist;
            setmodelstate.request.model_state=modelstate;
            sleep (1);

            std::string yPoss = line.substr(0, line.find("\t"));
            line = line.substr(line.find("\t") + 1);
            y = atof(yPoss.c_str());

            pose.position.x = getmodelstate.response.pose.position.x + getphysicsproperties.response.time_step*twist.linear.x + x;
            pose.position.y = getmodelstate.response.pose.position.y + getphysicsproperties.response.time_step*twist.linear.y + y;
            pose.position.z = getmodelstate.response.pose.position.z + getphysicsproperties.response.time_step*twist.linear.z + z;
            pose.orientation.x = 0.0;
            pose.orientation.y = 0.0;
            pose.orientation.z = 0.0;


            modelstate.pose = pose;
            modelstate.twist = twist;

            setmodelstate.request.model_state=modelstate;

            std::string zPoss = line.substr(0, line.find("\t"));
            line = line.substr(line.find("\t") + 1);
            z = atof(zPoss.c_str());

            pose.position.x = getmodelstate.response.pose.position.x + getphysicsproperties.response.time_step*twist.linear.x + x;
            pose.position.y = getmodelstate.response.pose.position.y + getphysicsproperties.response.time_step*twist.linear.y + y;
            pose.position.z = getmodelstate.response.pose.position.z + getphysicsproperties.response.time_step*twist.linear.z + z;
            pose.orientation.x = 0.0;
            pose.orientation.y = 0.0;
            pose.orientation.z = 0.0;
            modelstate.pose = pose;
            modelstate.twist = twist;



          //  gazebo_msgs::ModelState modelstate;
        //    modelstate.model_name ="Object_Black_Cylinder1";
            //modelstate.pose = pose;
            //modelstate.twist = twist;

            setmodelstate.request.model_state=modelstate;

            if (client.call(setmodelstate))
            {
               ROS_INFO("PERFECT!!!");
               ROS_INFO("%f",modelstate.pose.position.x);
               ROS_INFO("%f",modelstate.pose.position.y);
               ROS_INFO("%f",modelstate.pose.position.z);

            }
            else
            {
               ROS_ERROR("Failed to call service for setmodelstate ");
               return 1;
            }
            std::cout << "Receives:" << std::endl << "x:" << x << " y:" << y << " z:" << z << std::endl;
        }

	}
    else
    {
    	std::cout << "ERROR: Could not open trajectory file"<< std::endl;
    }

     return 0;
   }
     
