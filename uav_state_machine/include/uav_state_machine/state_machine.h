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
#include <thread>
#include <std_msgs/Float64.h>
#include <uav_state_machine/target_service.h>
#include <uav_state_machine/takeoff_service.h>
#include <uav_state_machine/land_service.h>
#include <uav_state_machine/waypoint_service.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>

#include <grvc_utils/argument_parser.h>
#include <grvc_quadrotor_hal/types.h>
#include <grvc_quadrotor_hal/server.h>
using namespace std;
using namespace grvc::hal;
using namespace grvc::com;
using namespace grvc::utils;
using namespace grvc;

namespace mbzirc{
    class Candidate;
    class CandidateList;
}

class UavStateMachine{
public:
    enum class eState { REPOSE, TAKINGOFF, HOVER, SEARCHING, CATCHING, LAND };

    bool Init(grvc::utils::ArgumentParser _args);

    void step();

private:

    void reposeCallback();
    bool searchingCallback(uav_state_machine::waypoint_service::Request &req, uav_state_machine::waypoint_service::Response &res);
    void catchingCallback();
    void candidateCallback(const std_msgs::String::ConstPtr& _msg);
    void joystickCb(const sensor_msgs::Joy::ConstPtr& _joy);
    void positionCallback(const geometry_msgs::PoseStamped::ConstPtr& _msg);
    void altitudeCallback(const std_msgs::Float64::ConstPtr& _msg);
    bool targetServiceCallback(uav_state_machine::target_service::Request  &req,
         uav_state_machine::target_service::Response &res);
    bool takeoffCallback(uav_state_machine::takeoff_service::Request  &req,
         uav_state_machine::takeoff_service::Response &res);
    bool landCallback(uav_state_machine::land_service::Request  &req,
         uav_state_machine::land_service::Response &res);

    bool bestCandidateMatch(const mbzirc::CandidateList &_list, const mbzirc::Candidate &_specs, mbzirc::Candidate &_matchedCandidate);

    grvc::hal::Server::PositionErrorService::Client *pos_error_srv;

    grvc::hal::Server::TakeOffService::Client* takeOff_srv;
    grvc::hal::Server::LandService::Client* land_srv;
    grvc::hal::Server::WaypointService::Client* waypoint_srv;
    grvc::hal::Server::AbortService::Client* abort_srv;
    
    ros::ServiceServer target_service;   
    ros::ServiceServer takeoff_service;  
    ros::ServiceServer land_service;
    ros::ServiceServer waypoint_service; 

    ros::Subscriber candidateSubscriber;
    ros::Subscriber joystickSubscriber;
    ros::Subscriber positionSubs;
    ros::Subscriber altitudeSubs;

    float mCurrentAltitude = 0;
    float mFlyTargetAltitude = 0;

    Waypoint mCurrentWaypoint;
    std::vector<Waypoint> mWaypointList;  
    int mWaypointItem = 0;

    eState mState =eState::REPOSE;
    mbzirc::Candidate *mTarget;

};
#endif
