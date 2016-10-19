#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandHome.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Joy.h>

#include <string>
#include <stdio.h>

#define MAX_VEL_LIN 1
#define MAX_VEL_ANG 1

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

sensor_msgs::NavSatFix position;
void home_cb(const sensor_msgs::NavSatFix::ConstPtr & global_pos)
{
    position = *global_pos;
   // ROS_INFO("Recieved pose: [%f %f %f]\n", position.latitude, position.longitude, position.altitude);
}

geometry_msgs::PoseStamped pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr & position)
{
    pose = *position;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 1;
    pose.pose.orientation.w = 0;

}

geometry_msgs::TwistStamped velocity;
void joy_cb(const sensor_msgs::Joy::ConstPtr & joy)
{
    velocity.twist.linear.x = -joy->axes[3] * MAX_VEL_LIN;
    velocity.twist.linear.y = joy->axes[4] * MAX_VEL_LIN;
    velocity.twist.linear.z = joy->axes[1] * MAX_VEL_LIN;
    velocity.twist.angular.z = joy->axes[0] * MAX_VEL_ANG;

}

int main(int argc, char **argv)
{
    int i;

    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber pos_home_gps = nh.subscribe<sensor_msgs::NavSatFix>
            ("mavros/global_position/global", 10, home_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::ServiceClient set_home_client = nh.serviceClient<mavros_msgs::CommandHome>
            ("mavros/cmd/set_home");
    ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>
            ("/joy", 10, joy_cb);


    ros::Rate rate (20.0);

    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    velocity.twist.linear.x = 0;
    velocity.twist.linear.y = 0;
    velocity.twist.linear.z = 0;
    velocity.twist.angular.x = 0;
    velocity.twist.angular.y = 0;
    velocity.twist.angular.z = 0;

    for(i = 100; ros::ok() && i > 0; --i)
    {
        vel_pub.publish(velocity);
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

   // ros::Time last_request = ros::Time::now();
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    offb_set_mode.response.success = false;

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    arm_cmd.response.success = false;

    mavros_msgs::CommandHome home_cmd;
    home_cmd.request.current_gps = true;
    home_cmd.request.latitude = 0;
    home_cmd.request.longitude = 0;
    home_cmd.request.altitude = 0;

    if(set_home_client.call(home_cmd))
    {
        if(home_cmd.response.success)
        {
            ROS_INFO ("SUCCESS");
        }
        if(home_cmd.response.result > 0)
        {
            ROS_INFO("NOT SET HOME");
            ROS_INFO("[%i]", home_cmd.response.result);
        }
        else
        {
            ROS_INFO("SET HOME");
            ROS_INFO("[%f %f %f]", home_cmd.request.latitude, home_cmd.request.longitude, home_cmd.request.altitude);
        }
    }
    else
    {
        ROS_INFO("FAILED!!");
    }

    bool initialized = false;
    while(ros::ok())
    {
        vel_pub.publish(velocity);

        if(!initialized) {

            if(!offb_set_mode.response.success) {
                set_mode_client.call(offb_set_mode);
            }

            if(!arm_cmd.response.success) {
                arming_client.call(arm_cmd);
            }

            if(current_state.mode == "OFFBOARD" && current_state.armed) {
                initialized = true;
                ROS_INFO("Initialized");

                for (i = 0; i < 100; i++)
                {
                    local_pos_pub.publish(pose);
                    ros::spinOnce();
                    usleep(10000);
                    ROS_INFO("Published");
                }
            }
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;

}
