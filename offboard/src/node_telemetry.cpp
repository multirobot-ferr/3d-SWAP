#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

sensor_msgs::NavSatFix g_position;
void global_cb(const sensor_msgs::NavSatFix::ConstPtr & global_position)
{
    g_position = *global_position;
    ROS_INFO("Recieved global pose: [%f %f %f]\n", g_position.latitude, g_position.longitude, g_position.altitude);
}

geometry_msgs::PoseStamped l_position;
void local_cb(const geometry_msgs::PoseStamped::ConstPtr & local_position)
{
    l_position = *local_position;
    ROS_INFO("Recieved local pose: [%f %f %f]\n", l_position.pose.position.x, l_position.pose.position.y, l_position.pose.position.z);
}

nav_msgs::Odometry l_UTM_position;
void local_UTM_cb(const nav_msgs::Odometry::ConstPtr & local_UTM_position)
{
    l_UTM_position = *local_UTM_position;
    ROS_INFO("Recieved UTM local pose: [%f %f %f]\n", l_UTM_position.pose.pose.position.x, l_UTM_position.pose.pose.position.y, l_UTM_position.pose.pose.position.z);
}

std_msgs::Float64 r_altitude;
void r_altitude_cb(const std_msgs::Float64::ConstPtr & relative_altitude)
{
    r_altitude = *relative_altitude;
    ROS_INFO("Recieved relative altitude: [%f]\n", r_altitude.data);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "telemetry_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 1000, state_cb);
    ros::Subscriber global_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("mavros/global_position/global", 1000, global_cb);
    ros::Subscriber local_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 1000, local_cb);
    ros::Subscriber localUTM_sub = nh.subscribe<nav_msgs::Odometry>
            ("mavros/global_position/local", 1000, local_UTM_cb);
    ros::Subscriber rel_altitude_sub = nh.subscribe<std_msgs::Float64>
            ("mavros/global_position/rel_alt", 1000, r_altitude_cb);

    ros::spin();

    return 0;

}
