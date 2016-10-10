#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}


int main(int argc, char **argv)
{
    int i;

    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    ros::Rate rate (20.0);

    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 3;
    pose.pose.position.z = 5;

   /* for(i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }*/

   // ros::Time last_request = ros::Time::now();
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    offb_set_mode.response.success = false;

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    arm_cmd.response.success = false;

    bool initialized = false;
    while(ros::ok())
    {
        local_pos_pub.publish(pose);

        if(!initialized) {

            if(!offb_set_mode.response.success) {
                set_mode_client.call(offb_set_mode);
                sleep(1);
            }
            if(!arm_cmd.response.success) {
                arming_client.call(arm_cmd);
                sleep(1);
            }

            if(current_state.mode == "OFFBOARD" && current_state.armed) {
                initialized = true;
                ROS_INFO("Initialized");
            }
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;

}
