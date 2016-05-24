/*
 * File for change positions of UAVs
 * Author: Luis Manuel Ramirez de la Cova
 * Date: April 2016
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

  int main(int argc, char** argv)
   {

    ros::init(argc, argv, "pruebamover");

    ros::NodeHandle n;

    sleep(20);

    
    ros::ServiceClient gpp_c = n.serviceClient<gazebo_msgs::GetPhysicsProperties>("/gazebo/get_physics_properties");
    gazebo_msgs::GetPhysicsProperties getphysicsproperties;
    gpp_c.call(getphysicsproperties);  

    double t=0.1;

    t=getphysicsproperties.response.time_step + t;

    ros::ServiceClient gms_c = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    gazebo_msgs::GetModelState getmodelstate;
    gms_c.call(getmodelstate);

    geometry_msgs::Twist twist;
    twist.linear.x = 0.01;
    twist.linear.y = 0.01;
    twist.linear.z = 0.01;
    twist.angular.x = 0.01;
    twist.angular.y = 0.01;
    twist.angular.z = 0.01;

    geometry_msgs::Pose pose;
    pose.position.x = getmodelstate.response.pose.position.x + getphysicsproperties.response.time_step*twist.linear.x;
    pose.position.y = getmodelstate.response.pose.position.y + getphysicsproperties.response.time_step*twist.linear.y;
    pose.position.z = getmodelstate.response.pose.position.z+1.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;

  	ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state"); 
    gazebo_msgs::SetModelState setmodelstate;

    gazebo_msgs::ModelState modelstate;
    modelstate.model_name ="Object_Black_Cylinder1";
    modelstate.pose = pose;
    modelstate.twist = twist;

    setmodelstate.request.model_state=modelstate;  

    

     if (client.call(setmodelstate))
      {
       ROS_INFO("PERFECT!!!");
       ROS_INFO("%f",getphysicsproperties.response.time_step);
       ROS_INFO("%f",modelstate.pose.position.x);
       ROS_INFO("%f",modelstate.pose.position.y);
       ROS_INFO("%f",modelstate.twist.linear.x);
       ROS_INFO("%f",getmodelstate.response.pose.position.y);
      }
     else
      {
       ROS_ERROR("Failed to call service for setmodelstate ");
       return 1;
      }



     return 0;
   }
   
    /* int main(int argc, char** argv)
   {

    ros::init(argc, argv, "pruebamover");

    ros::NodeHandle n;

    sleep(10);

    
    ros::ServiceClient gpp_c = n.serviceClient<gazebo_msgs::GetPhysicsProperties>("/gazebo/get_physics_properties");
    gazebo_msgs::GetPhysicsProperties getphysicsproperties;
    gpp_c.call(getphysicsproperties);  

    double t=0.1;

    t=getphysicsproperties.response.time_step + t;

    ros::ServiceClient gms_c = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    gazebo_msgs::GetModelState getmodelstate;
    gms_c.call(getmodelstate);

    geometry_msgs::Twist twist;
    twist.linear.x = 2.0;
    twist.linear.y = 2.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;

    geometry_msgs::Pose pose;
    pose.position.x = getmodelstate.response.pose.position.x + getphysicsproperties.response.time_step*twist.linear.x;
    pose.position.y = getmodelstate.response.pose.position.y+3.0;
    pose.position.z = getmodelstate.response.pose.position.z+4.5;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;

  	ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state"); 
    gazebo_msgs::SetModelState setmodelstate;

    gazebo_msgs::ModelState modelstate;
    modelstate.model_name ="uav1";
    modelstate.pose = pose;
    modelstate.twist = twist;

    setmodelstate.request.model_state=modelstate;  

    

     if (client.call(setmodelstate))
      {
       ROS_INFO("PERFECT!!!");
       ROS_INFO("%f",getphysicsproperties.response.time_step);
       ROS_INFO("%f",modelstate.pose.position.x);
       ROS_INFO("%f",modelstate.pose.position.y);
       ROS_INFO("%f",modelstate.twist.linear.x);
       ROS_INFO("%f",getmodelstate.response.pose.position.y);
      }
     else
      {
       ROS_ERROR("Failed to call service for setmodelstate ");
       return 1;
      }



     return 0;
   }*/
