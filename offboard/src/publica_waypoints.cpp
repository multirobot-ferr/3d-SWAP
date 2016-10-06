#include "ros/ros.h"
#include "std_msgs/String.h"
#include "mavros_msgs/WaypointPush.h"
#include <iostream>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener_mavlink");
  ros::NodeHandle n;

  mavros_msgs::WaypointPush service;
  mavros_msgs::Waypoint wp;

  //Posicion inicial del home
  wp.command=16;
  //wp.frame=0;
  wp.is_current=true;
  wp.autocontinue=true;
  /*wp.param1=0;
  wp.param2=0;
  wp.param3=0;
  wp.param4=0;**/
  wp.x_lat=47.3977418;
  wp.y_long=8.5455938;
  wp.z_alt=489.0;
  service.request.waypoints.push_back(wp);


  //Takeoff (quitarlo si quieres)
 /* wp.frame=3;
  wp.command=22;
  wp.is_current=true;
  wp.autocontinue=true;
  wp.param1=0;
  wp.param2=0;
  wp.param3=0;
  wp.param4=0;
  wp.x_lat=0;//47.3977418;
  wp.y_long=0;//8.5455938;
  wp.z_alt=10;*/
  //service.request.waypoints.push_back(wp);


  //Waypoint 1
  wp.frame=3;
  wp.command=16;
  wp.is_current=false;
  wp.autocontinue=true;
  wp.param1=0;
  wp.param2=0;
  wp.param3=0;
  wp.param4=0;
  wp.x_lat = 47.3974360;
  wp.y_long = 8.5450240;
  wp.z_alt=10;
  service.request.waypoints.push_back(wp);

  //Waypoint 2
  wp.frame=3;
  wp.command=16;
  wp.is_current=false;
  wp.autocontinue=true;
  wp.param1=0;
  wp.param2=0;
  wp.param3=0;
  wp.param4=0;
  wp.x_lat = 47.3972510;
  wp.y_long = 8.5449650;
  wp.z_alt=10;
  service.request.waypoints.push_back(wp);

  //Waypoint 3
  wp.frame=3;
  wp.command=16;
  wp.is_current=false;
  wp.autocontinue=true;
  wp.param1=0;
  wp.param2=0;
  wp.param3=0;
  wp.param4=0;
  wp.x_lat = 47.3979030;
  wp.y_long = 8.5456310;
  wp.z_alt=10;
  service.request.waypoints.push_back(wp);

  //Waypoint 4
  wp.frame=3;
  wp.command=16;
  wp.is_current=false;
  wp.autocontinue=true;
  wp.param1=0;
  wp.param2=0;
  wp.param3=0;
  wp.param4=0;
  wp.x_lat = 47.3976770;
  wp.y_long = 8.5428140;
  wp.z_alt=10;
  service.request.waypoints.push_back(wp);



 /** wp.frame=10;
  wp.command=16;
  wp.is_current=false;
  wp.autocontinue=false;
  wp.param1=15;
  wp.param2=0;
  wp.param3=0;
  wp.param4=0;
  wp.x_lat=-35.3595162;
  wp.y_long=149.1533518;
  wp.z_alt=200.8;
  service.request.waypoints.push_back(wp);**/





  ros::ServiceClient sclient=n.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
  if(sclient.call(service))
    ROS_INFO("Servicio realizado con exito");
  else
    ROS_INFO("Servicio realizado sin exito");



  return 0;
}
