//------------------------------------------------------------------------------
// GRVC MBZIRC Vision
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
#ifndef _MBZIRC_STATEMACHINE_H_
#define _MBZIRC_STATEMACHINE_H_

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>

#include <uav_state_machine/hal_client.h>
#include <uav_state_machine/catching_device.h>
#include <uav_state_machine/candidate_list.h>
#include <uav_state_machine/candidate.h>
#include <uav_state_machine/uav_state.h>
#include <uav_state_machine/target_service.h>
#include <uav_state_machine/takeoff_service.h>
#include <uav_state_machine/land_service.h>
#include <uav_state_machine/waypoint_service.h>

#include <grvc_utils/argument_parser.h>

class UavStateMachine : public HalClient {
public:
    UavStateMachine(grvc::utils::ArgumentParser _args);

    bool init();
    void step();

private:
    bool takeoffServiceCallback(uav_state_machine::takeoff_service::Request  &req,
         uav_state_machine::takeoff_service::Response &res);
    bool landServiceCallback(uav_state_machine::land_service::Request  &req,
         uav_state_machine::land_service::Response &res);
    bool searchServiceCallback(uav_state_machine::waypoint_service::Request &req,
         uav_state_machine::waypoint_service::Response &res);
    bool targetServiceCallback(uav_state_machine::target_service::Request  &req,
         uav_state_machine::target_service::Response &res);

    void positionCallback(const geometry_msgs::PoseStamped::ConstPtr& _msg);
    void altitudeCallback(const std_msgs::Float64::ConstPtr& _msg);
    void joyCallback(const sensor_msgs::Joy::ConstPtr& _joy);

    void onSearching();
    void onCatching();

    void candidateCallback(const uav_state_machine::candidate_list::ConstPtr& _msg);
    bool bestCandidateMatch(const uav_state_machine::candidate_list, const uav_state_machine::candidate &_specs, uav_state_machine::candidate &_result);
    
    CatchingDevice *catching_device_;
    ros::ServiceServer take_off_service_;
    ros::ServiceServer land_service_;
    ros::ServiceServer search_service_;
    ros::ServiceServer target_service_;

    ros::Subscriber position_sub_;
    ros::Subscriber altitude_sub_;
    ros::Subscriber joy_sub_;

    uav_state_machine::uav_state state_;

    grvc::hal::Waypoint current_position_waypoint_;
    std::vector<grvc::hal::Waypoint> waypoint_list_;
    unsigned int waypoint_index_ = 0;
    float current_altitude_ = 0;
    float target_altitude_ = 0;
    uav_state_machine::candidate target_;
};

#endif  // _MBZIRC_STATEMACHINE_H_
