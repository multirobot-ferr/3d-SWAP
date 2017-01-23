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

#include <ros/ros.h>
#include <thread>
#include <CandidateList.h>
#include <Candidate.h>
#include <std_msgs/Float64.h>
#include <uav_state_machine/target_service.h>
#include <uav_state_machine/takeoff_service.h>
#include <uav_state_machine/land_service.h>
#include <sensor_msgs/Joy.h>

#include <grvc_utils/argument_parser.h>
#include <grvc_quadrotor_hal/types.h>
#include <grvc_quadrotor_hal/server.h>


class UavStateMachine{
public:
    enum class eState { REPOSE, TAKINGOFF, HOVER, CATCHING, LAND }

    UavStateMachine(int _argc, char** _argv);
    ~UavStateMachine();

    void step();

private:

    void catching();
    void candidateCallback(const std_msgs::String::ConstPtr _msg);
    void joystickCb(const sensor_msgs::Joy::ConstPtr& _joy);
    void altitudeCallback(const std_msgs::Float64::ConstPtr _msg);
    bool targetCallback(uav_visual_servoing::target_service::Request  &req,
         uav_visual_servoing::target_service::Response &res);
    bool takeoffCallback(uav_visual_servoing::takeoff_service::Request  &req,
         uav_visual_servoing::takeoff_service::Response &res);
    bool landCallback(uav_visual_servoing::land_service::Request  &req,
         uav_visual_servoing::land_service::Response &res);

    hal::Server::PositionErrorService::Client *pos_error_srv;

    hal::Server::TakeOffService::Client* takeOff_srv;
    hal::Server::LandService::Client* land_srv;

    ros::ServiceServer target_service;   
    ros::ServiceServer takeoff_service;  
    ros::ServiceServer land_service;    
    ros::Subscriber candidateSubscriber;
    ros::Subscriber joystickSubscriber;

    
    float mAltitude = 15;
    float mFlyTargetAltitude = 15;  

    eState mState;
    Candidate mTarget;

}