#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <cmath>

int main (int argc, char** argv)
{
    ros::init(argc, argv, "marker_waypoints");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("marker_waypoints", 10);

    ros::Rate rate(30);

    float f = 0.0;

    while(ros::ok())
    {
        visualization_msgs::Marker points, line_strip, line_list;
        points.header.frame_id = line_strip.header.frame_id = "/map";
        points.header.stamp = line_strip.header.stamp = ros::Time::now();
        points.ns = line_strip.ns = "points_and_lines";
        points.action = line_strip.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

        points.id = 0;
        line_strip.id = 1;

        points.type = visualization_msgs::Marker::POINTS;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;


        //Points markers use x and y scale for width/height respectively
        points.scale.x = 2;
        points.scale.y = 2;

        //Line_Strip/Line_List markers use only the x component of scale, for the line width
        line_strip.scale.x = 0.1;
        line_list.scale.y = 0.1;

        //Points are green
        points.color.g = 1.0f;
        points.color.a = 1.0;

        //Line_Strip are blue
        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;


        //Create the vertices for the points and lines

        geometry_msgs::Point p;
        p.x = 60;
        p.y = -25;
        p.z = 20;

        points.points.push_back(p);
        line_strip.points.push_back(p);

        marker_pub.publish(points);
        marker_pub.publish(line_strip);


        rate.sleep();

        f+= 0.04;
    }
}
