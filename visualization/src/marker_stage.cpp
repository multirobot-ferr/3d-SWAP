#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <list>

using namespace std;

void publishStage();

map<string,ros::Publisher> vis_models;
ros::Publisher vis_world;


int main(int argc, char** argv)
{

    ros::init(argc, argv, "marker_stage");

    ros::NodeHandle n;

    vis_world = n.advertise<visualization_msgs::Marker>("visualization_world", 0);

    ros::Rate loop(100);
    while(ros::ok())
    {
        ros::spinOnce();
        publishStage();
        loop.sleep();
    }

}

void publishStage()
{
    //Visualize the stage
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();
    marker.ns = "marker_stage";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://mbzirc_gcs_view/model/stage_01.dae";
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;

    marker.mesh_use_embedded_materials = true;

  /*  marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;*/
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    vis_world.publish(marker);

}
