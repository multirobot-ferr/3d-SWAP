/*
 * Copyright (c) 2017, swap-ferr
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of swap-ferr nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//#include "swap_2_5d.h"
#include <uav_avoidance/swap_2_5d.h>

#include <iostream>

int main(int argc, char **argv) {
    // name remapping
    ros::init(argc, argv, "swap_2_d");

    avoid::Swap s;
    int a = 3;

    Swap_2_5d swap;

//    SwapROS swap;
//
//    swap.SetGranularity(180);
//    swap.SetMeasurementsPerDirection(5);
//    swap.SetMultiInputPond(0.5);
//
//    swap.SetSafetyRegion(0.26);
//    swap.SetBrackingDistance(0.2);
//    swap.SetRangerError(0.01);
//    swap.SetGammaOffset(0.0);
//    swap.SetSmoothFactor(5);
////    swap.SetGoalLateralVision(90.0);
//
    std::string why;
    if (true)//!swap.IsReady(why))
    {
        ROS_FATAL("%s", why.c_str());
    }
    else
    {
        while (ros::ok()){
//            std::string state;
//            if (swap.GetMachineStateChanges(state))
//                ROS_INFO("Swap: %s", state.c_str());
//
//            ros::spinOnce();
//
//            // Performs all necessary publications
//            swap.Publish();
//
//
//            // Sleeping to save time (it should wait at least for 3 laser scans)
//            ros::Duration(0.3).sleep();
        }
    }


    return 0;
}

///**
// * Default constructor of the class
// */
//SwapROS::SwapROS()
//{
//    // Preparing the laser (if available)
//    std::string laser_topic = "base_scan";
//    int         laser_topic_queue = 3;
//    if (nh_.getParam("swap/laser_topic", laser_topic)){
//        nh_.param("swap/laser_topic_queue", laser_topic_queue, 3);
//        nh_.param("swap/laser_asume_dynamic", laser_asume_dynamic_, true);
//        laser_sub_ = nh_.subscribe(laser_topic.c_str(), laser_topic_queue, &SwapROS::LaserReceived, this);
//        ROS_INFO("SwapROS: Connected to %s, queue %d, assuming %s measurements",
//                 laser_topic.c_str(), laser_topic_queue, laser_asume_dynamic_ ? "dynamic" : "static");
//    }else{
//        ROS_WARN("SwapROS: No laser connected");
//    }
//
//    // Subscribing to a goal
//    std::string goal_topic = "move_base_simple/goal";
//    nh_.param("swap/goal_topic", goal_topic, std::string("move_base_simple/goal"));
//    goal_sub_ = nh_.subscribe( goal_topic.c_str(), 1, &SwapROS::GoalReceived, this);
//    ROS_INFO("SwapROS: Connected to %s to receive goals", goal_topic.c_str());
//
//    // Subscribing to the position
//    std::string pose_topic = "amcl_pose";
//    nh_.param("swap/position_topic", pose_topic, std::string("amcl_pose"));
//    pose_sub_ = nh_.subscribe( pose_topic.c_str(), 1, &SwapROS::PoseReceived, this);
//    ROS_INFO("SwapROS: Connected to %s to receive positions", pose_topic.c_str());
//
//    nh_.param( "swap/debug", debug_, false);
//    if (debug_){
//        ROS_WARN("SwapROS: Debug modus activated. The computational efficiency could be compromised");
//        debug_pub_ = nh_.advertise<PLCDebug::PointCloud>("swap/debug",1);
//
//        if (!ros::param::get( "swap/debug_frame_id", debug_frame_id_))
//        {
//            debug_frame_id_ = "/";
//        }
//
//        ROS_WARN("%s", debug_frame_id_.c_str());
//    }
//
//    // Neccesary information to command the robot
//    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel",2 , false);
//
//    ROS_INFO("SwapROS: Init complete");
//}
//
///**
// * Performs all necessary publications
// */
//void SwapROS::Publish()
//{
//    PublishCmdVel();
//
//    if (debug_)
//    {
//        PublishDebugPCL();
//    }
//}
//
///**
// * Callback for a laser ranger message
// */
//void SwapROS::LaserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan_msg)
//{
//    NewMeasurementSetReceived();
//    for (unsigned id_laser = 0; id_laser < laser_scan_msg->ranges.size(); ++id_laser)
//    {
//        double angle = laser_scan_msg->angle_min + double(id_laser)*laser_scan_msg->angle_increment;
//        SetNewLocalMeasurement( laser_scan_msg->ranges[id_laser], angle, false); //TODO laser_asume_dynamic_);
//    }
//}
//
//void SwapROS::GoalReceived(const geometry_msgs::PoseStamped& goal_msg)
//{
//    goal_x_   = goal_msg.pose.position.x;
//    goal_y_   = goal_msg.pose.position.y;
//    goal_yaw_ = tf::getYaw(goal_msg.pose.orientation);
//
//    if ( !goal_msg.header.frame_id.find("map") ){
//        // Goal received with respect to the map
//        goal_map_received_   = true;
//        goal_robot_received_ = false;
//
//        ROS_INFO("SwapROS: Goal (respect to map): %f,%f,%f", goal_x_ , goal_y_ , goal_yaw_*180/M_PI);
//    }
//    else if ( !goal_msg.header.frame_id.find( "base_link") )
//    {
//        // Goal received with respect to the robot
//        goal_map_received_   = false;
//        goal_robot_received_ = true;
//
//        ROS_INFO("SwapROS: Goal (respect to robot): %f,%f,%f", goal_x_ , goal_y_, goal_yaw_*180/M_PI);
//    }
//    else
//    {
//        ROS_WARN("SwapROS: Goal not recognized. Change your frame_id");
//        goal_map_received_   = false;
//        goal_robot_received_ = false;
//    }
//
//    CommandGoals();
//}
//
//void SwapROS::PoseReceived(const geometry_msgs::PoseWithCovarianceStamped& pose_msg)
//{
//    if ( !pose_msg.header.frame_id.find( "map" ) ||
//         !pose_msg.header.frame_id.find( "base_link" )){
//        // Pose received with respect to the map or the robot.
//
//        robot_x_   = pose_msg.pose.pose.position.x;
//        robot_y_   = pose_msg.pose.pose.position.y;
//        robot_yaw_ = tf::getYaw(pose_msg.pose.pose.orientation);
//
//        // // Convert quaternion to roll-pitch-yaw
//        // double roll, pitch, yaw;
//        // tf::Quaternion q(msg->pose.pose.orientation.x,
//        //                  msg->pose.pose.orientation.y,
//        //                  msg->pose.pose.orientation.z,
//        //                  msg->pose.pose.orientation.w);
//        // tf::Matrix3x3 m(q);
//        // m.getRPY(roll, pitch, yaw);
//
//        pose_received_ = true;
//    }
//
//    CommandGoals();
//}
//
//
///**
// * Computes the necessary control actions for the robot and publishes them
// */
//void SwapROS::PublishCmdVel()
//{
//    // Computing the necessary references
//    CollisionAvoidance(v_ref_, yaw_ref_);
//
//    // prepare the cmd_vel message according to the commanded accelerations
//    geometry_msgs::Twist cmd_vel;
//
//    //    ros::Duration t_incr = ros::Time::now() - RefCmdTime_;
//    //    if (vRef_ > vRefCmd_){
//    //        vRefCmd_ = std::min(vRef_, vRefCmd_ + accelCmd_*t_incr.toSec());
//    //    }else if (vRef_ < vRefCmd_){
//    //        vRefCmd_ = std::max(vRef_, vRefCmd_ - decelCmd_*t_incr.toSec());  // TODO: Tose values has to be used by the user
//    //                                                                          // TODO: We need a forced version of the cmdvel
//    //    }
//
//    //    cmd_vel.linear.x = vRefCmd_;
//    cmd_vel.linear.x = v_ref_;
//    cmd_vel.linear.y = 0.0;
//    cmd_vel.linear.z = 0.0;
//    cmd_vel.angular.x = 0.0;
//    cmd_vel.angular.y = 0.0;
//    cmd_vel.angular.z = yaw_ref_;
//    //    cmd_vel.angular.z = yawPcmd_*yawRef_;      // TODO: PID controler
//
//    // publish new command velocity
//    cmd_vel_pub_.publish(cmd_vel);
//    //    RefCmdTime_ = ros::Time::now();
//}
//
///**
// * Publish a debuging PointCloud
// */
//void SwapROS::PublishDebugPCL()
//{
//    using namespace PLCDebug;
//
//    // Preparing the message and the header
//    static PointCloud::Ptr pointCloud_msg (new PointCloud);        // msg definition (static to avoid multiple-allocations)
//    pcl_conversions::toPCL(ros::Time::now(), pointCloud_msg->header.stamp);
//    pointCloud_msg->header.frame_id = debug_frame_id_;
//    pointCloud_msg->points.clear();
//
//    // Add points for the reserved disk and the polar obstacle diagram
//    pcl::PointXYZRGB measurement_po     = {0,0,0};
//    pcl::PointXYZRGB safety_region_po   = {0,0,0};
//    pcl::PointXYZRGB conflicts_po       = {0,0,0};
//    pcl::PointXYZRGB reference_po       = {0,0,0};
//    PolarObstacleDiagram::measurements info;
//
//    // Setting fixed colors
//    SetPCLcolor(measurement_po,     DARK_BLUE);
//    SetPCLcolor(safety_region_po,   GREEN);
//    SetPCLcolor(conflicts_po,       RED);
//    SetPCLcolor(reference_po,       BLACK);
//
//    // Setting the measurements and the safety_region around the robot
//    for (unsigned id_phi = 0; id_phi < id_phi_max_; ++id_phi)
//    {
//        info = GetMeasurement( id_phi );
//
//        measurement_po.x = info.dist * cos( info.angle );
//        measurement_po.y = info.dist * sin( info.angle );
//
//        safety_region_po.x = info.safety_region * cos( info.angle );
//        safety_region_po.y = info.safety_region * sin( info.angle );
//
//        pointCloud_msg->points.push_back (measurement_po);
//        pointCloud_msg->points.push_back (safety_region_po);
//    }
//
//    // Showing the conflicts (if any)
//    for (unsigned id_conf = 0; id_conf < conflictive_angles_.size(); ++id_conf)
//    {
//        for (double d = 0.0; d < 1.0; d+=0.1)
//        {
//            conflicts_po.x = d*cos( conflictive_angles_[id_conf]);
//            conflicts_po.y = d*sin( conflictive_angles_[id_conf]);
//
//            pointCloud_msg->points.push_back (conflicts_po);
//        }
//    }
//
//    // Showing the commanded direction
//    for (double d = 0.0; d < 1.0; d+=0.1)
//    {
//        reference_po.x = d*cos( yaw_ref_ );
//        reference_po.y = d*sin( yaw_ref_ );
//
//        pointCloud_msg->points.push_back (reference_po);
//    }
//
//
//    pointCloud_msg->height = 1;
//    pointCloud_msg->width  = pointCloud_msg->points.size();
//
//    debug_pub_.publish(pointCloud_msg);
//}
//
///**
// *  Converts from different frame id to submit the necessary goal position and orientation
// */
//void SwapROS::CommandGoals()
//{
//    double x_robot2goal, y_robot2goal;
//    double dist2goal = 0.0,  ang2goal = 0.0;  // If somethings goes wrong, robot does not move
//
//    if ( goal_map_received_ && pose_received_ )
//    {
//        // Computing goal with respect to the current position
//        double diff_x = goal_x_ - robot_x_;
//        double diff_y = goal_y_ - robot_y_;
//
//        // Rotate from global coordinate system to local
//        x_robot2goal = diff_x*cos(-robot_yaw_) - diff_y*sin(-robot_yaw_);
//        y_robot2goal = diff_x*sin(-robot_yaw_) + diff_y*cos(-robot_yaw_);
//
//        dist2goal = sqrt( powf(x_robot2goal,2) + powf(y_robot2goal,2) );
//        ang2goal  = atan2(y_robot2goal, x_robot2goal);
//        SetGoal(dist2goal, ang2goal);
//    }
//    else if ( goal_robot_received_ )
//    {
//        x_robot2goal = goal_x_;
//        y_robot2goal = goal_y_;
//
//        dist2goal = sqrt( powf(x_robot2goal,2) + powf(y_robot2goal,2) );
//        ang2goal  = atan2(y_robot2goal, x_robot2goal);
//        SetGoal(dist2goal, ang2goal);
//    }
//    else
//    {
//        SetGoal( 0.0, 0.0);
//    }
//
//}
//
///**
// * Utility function. Set the color of the specified point cloud
// */
//void SwapROS::SetPCLcolor(pcl::PointXYZRGB& point, PLCDebug::PLC_COLOR color)
//{
//    point.r = color.R;
//    point.g = color.G;
//    point.b = color.B;
//}
