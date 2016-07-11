#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <stdio.h>
#include <std_msgs/String.h>
#include <string.h>


using namespace std;

/*void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    //ROS_INFO("[%s]", msg->data.c_str());

    int i = 0;
    char * str =(char*) msg->data.c_str();
    char * pch = strtok(str,"\n");
    string pos[3];

    while(pch != NULL)
    {
        //printf("%s\n",pch);
        pos[i++].append(pch);
        pch = strtok (NULL, "\n");

    }

    for(i=0; i<3; i++)
    {
        printf("%s\n",pos[2].c_str());
    }

}*/


int main(int argc, char** argv)
{
    ros::init(argc, argv, "take_object");
    ros::NodeHandle n;

   // ros::Subscriber sub = n.subscribe("/quad1/hal/position", 1000, chatterCallback);
    tf::TransformBroadcaster br;
    tf::Transform transform;

    ros::Rate rate(10);

    while(n.ok())
    {
        transform.setOrigin(tf::Vector3(0.0, 0.1, 0.0));
        transform.setRotation(tf::Quaternion(0, 0, 0, 1));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link_1", "base_link_4"));

        rate.sleep();
    }

    return 0;

   // ros::spin();

    //return 0;
}
