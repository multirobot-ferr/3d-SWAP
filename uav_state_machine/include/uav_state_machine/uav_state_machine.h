//------------------------------------------------------------------------------
// GRVC MBZIRC
//------------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2016 GRVC University of Seville
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//------------------------------------------------------------------------------
#ifndef MBZIRC_UAV_STATE_MACHINE_H
#define MBZIRC_UAV_STATE_MACHINE_H

#include <thread>
#include <ros/ros.h>
#include <Eigen/Core>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/PoseStamped.h>
#include <uav_state_machine/catching_device.h>
#include <uav_state_machine/CandidateList.h>
#include <uav_state_machine/Candidate.h>
#include <uav_state_machine/UavState.h>
#include <uav_state_machine/SetTarget.h>
#include <uav_state_machine/TakeOff.h>
#include <uav_state_machine/Land.h>
#include <uav_state_machine/TrackPath.h>
// @Capi
// #include <mbzirc_scheduler/SetTargetStatus.h>
#include <uav_abstraction_layer/ual.h>
#include <argument_parser/argument_parser.h>

class UavStateMachine {
public:
    UavStateMachine(grvc::utils::ArgumentParser _args);

    void init();
    void step();

private:
    bool takeoffServiceCallback(uav_state_machine::TakeOff::Request  &req,
         uav_state_machine::TakeOff::Response &res);
    bool landServiceCallback(uav_state_machine::Land::Request  &req,
         uav_state_machine::Land::Response &res);
    bool searchServiceCallback(uav_state_machine::TrackPath::Request &req,
         uav_state_machine::TrackPath::Response &res);
    bool targetServiceCallback(uav_state_machine::SetTarget::Request  &req,
         uav_state_machine::SetTarget::Response &res);

    void positionCallback(const geometry_msgs::Pose::ConstPtr& uav_pose);
    void lidarAltitudeCallback(const sensor_msgs::Range::ConstPtr& _msg);

    void onSearching();
    void onCatching();
    void onGoToDeploy();

    void candidateCallback(const uav_state_machine::CandidateList::ConstPtr& _msg);
    bool bestCandidateMatch(const uav_state_machine::CandidateList, const uav_state_machine::Candidate &_specs, uav_state_machine::Candidate &_result);

    int uav_id_ = -1;
    grvc::ual::UAL ual_;
    CatchingDevice *catching_device_;
    ros::ServiceServer take_off_service_;
    ros::ServiceServer land_service_;
    ros::ServiceServer search_service_;
    ros::ServiceServer target_service_;
    ros::ServiceClient target_status_client_;
    ros::ServiceClient deploy_approach_client_;
    ros::ServiceClient deploy_area_client_;
    ros::ServiceClient vision_algorithm_switcher_client_;

    ros::Subscriber position_sub_;
    ros::Subscriber altitude_sub_;
    ros::Subscriber lidar_altitude_sub_;
    ros::Publisher lidar_altitude_remapped_pub_;

    uav_state_machine::UavState state_;
    std::thread state_pub_thread_;
    ros::Publisher state_publisher_;

    uav_state_machine::Candidate matched_candidate_;
    Eigen::Matrix<double, 3, 1> target_position_ = { 0.0, 0.0, 0.0 };
    unsigned max_tries_counter_ = 3;

    grvc::ual::Waypoint current_position_waypoint_;  // Stores current position of the uav
    grvc::ual::Waypoint hover_position_waypoint_;
    std::vector<grvc::ual::Waypoint> waypoint_list_;
    unsigned int waypoint_index_ = 0;

    float lidar_range_ = 0;
    float current_altitude_ = 0;
    float target_altitude_ = 0;
    float flying_level_;
    uav_state_machine::SetTarget::Request target_;
    // @Arturo
    // grvc::utils::frame_transform frame_transform_;
};

#endif  // MBZIRC_UAV_STATE_MACHINE_H
