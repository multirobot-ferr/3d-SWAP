/*
 * Waypoint Publisher from File
 * Author: Luis Manuel Ramirez de la Cova
 * Created: April 2016
 * Modificate: 03-05-2016
 * Organization: University of Seville, GRVC
 * Description:
 */
 
#include <fstream>
#include <iostream>

#include <eigen3/Eigen/Geometry>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

bool sim_running = false;

static const int64_t kNanoSecondsInSecond = 1000000000;

void callback(const sensor_msgs::ImuPtr&)
{
    sim_running = true;
}

class WaypointWithTime
{
public:
    WaypointWithTime()
        : waiting_time(0){
    }
    WaypointWithTime(double t, float x, float y, float z)
        : position(x, y, z), waiting_time(t) {
    }

    Eigen::Vector3d position;
    double waiting_time;

};

int main (int argc, char** argv)
{
    ros::init(argc, argv, "waypoint_publisher");
    ros::NodeHandle nh;

    ROS_INFO("Started waypoint_publisher");

    ros::V_string args;
    ros::removeROSArgs(argc, argv, args);

    if(args.size() != 2 && args.size() != 3)
    {
        ROS_ERROR("Usage: waypoint_publisher <waypoint_file>" "\nThe waypoint file should be structured as: space separated: wait_time [s] x[m] y[m] z[m]");
        return -1;
    }

    std::vector<WaypointWithTime> waypoints;
    const float DEG_2_RAD = M_PI / 180.0;

    std::ifstream wp_file(args.at(1).c_str());
    wp_file.open("/home/luisgrvc/catkin_ws/src/mbzirc/resource/wp_file.txt");

    if(wp_file.is_open())
    {
        double t, x, y, z;

        while(wp_file >> t >> x >> y >> z)
        {
            waypoints.push_back(WaypointWithTime(t, x, y, z));
        }
        wp_file.close();
        ROS_INFO("Read %d waypoints.", waypoints.size());
    }
    else
    {
        ROS_ERROR_STREAM("Unable to open poses file: " << args.at(1));
        return -1;
    }

    //The IMU is used, to determine if the simulator is running or not
    ros::Subscriber sub = nh.subscribe("imu", 10, &callback);

    ros::Publisher wp = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

    ROS_INFO("Wait for simulation to become ready...");

    while(!sim_running && ros::ok())
    {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    ROS_INFO("...ok");

    //Wait for 30s such that everything can settle and the mav flies to the initial position
    ros::Duration(30).sleep();

    ROS_INFO("Start publishing waypoints.");

    trajectory_msgs::MultiDOFJointTrajectoryPtr msg(new trajectory_msgs::MultiDOFJointTrajectory);
    msg->header.stamp = ros::Time::now();
    msg->points.resize(waypoints.size());
    msg->joint_names.push_back("base_link");
    int64_t time_from_start_ns = 0;
    for (size_t i = 0; i < waypoints.size(); i++)
    {
        WaypointWithTime& wp = waypoints[i];

        mav_msgs::EigenTrajectoryPoint trajectory_point;
        trajectory_point.position_W = wp.position;
        trajectory_point.time_from_start_ns = time_from_start_ns;

        time_from_start_ns += static_cast<int64_t>(wp.waiting_time * kNanoSecondsInSecond);

        mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &msg->points[i]);
    }

    wp.publish(msg);

    return 0;
}
















