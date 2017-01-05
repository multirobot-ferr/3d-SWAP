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
#include <std_msgs/String.h>

#include <grvc_utils/argument_parser.h>
#include <grvc_quadrotor_hal/types.h>
#include <grvc_quadrotor_hal/server.h>

#include <thread>
#include <chrono>
#include <mutex>

#include <gcsLink.h>
#include <CandidateList.h>
#include <Candidate.h>
#include <std_msgs/Float64.h>


using namespace grvc;
using namespace std;

void candidateCallback(const std_msgs::String::ConstPtr _msg);
float gAltitude = 15;
void altitudeCallback(const std_msgs::Float64::ConstPtr _msg){
    gAltitude = _msg->data;
}
mbzirc::Candidate bestCandidateMatch(const mbzirc::CandidateList &_list, const mbzirc::Candidate &_specs);

hal::Server::PositionErrorService::Client *pos_error_srv;

int main(int _argc, char** _argv){

    std::cout << "Setting up" << std::endl;

    // Init services.
    grvc::utils::ArgumentParser args(_argc, _argv);
    pos_error_srv = new hal::Server::PositionErrorService::Client("/mbzirc_1/hal/pos_error", args);

    ros::Subscriber altitudeSubs;
    ros::NodeHandle nh;
    if(ros::isInitialized()){
        altitudeSubs = nh.subscribe<std_msgs::Float64>("/mavros_1/global_position/rel_alt", 10, altitudeCallback);
    }
    

    hal::Server::TakeOffService::Client takeOff_srv("/mbzirc_1/hal/take_off", args);
    hal::TaskState ts;
    while(!takeOff_srv.isConnected()) {
        this_thread::sleep_for(chrono::milliseconds(100));
    }

    double flyZ = 15.0;
    takeOff_srv.send(flyZ, ts);

    std::this_thread::sleep_for(std::chrono::seconds(5));
    std::cout << "Connected to hal" << std::endl;

    mbzirc::CandidateList candidateList;
    ros::Subscriber candidateSubscriber = nh.subscribe<std_msgs::String>("/candidateList", 1, candidateCallback);

    if(!candidateSubscriber){
        std::cout << ("Can't start candidate subscriber.") << std::endl;
        return -1;
    }else{
        std::cout << "Subscribed to candidate topic" << std::endl;
    }

    std::cout << "Initialized" << std::endl;

    while(true){

    }
}

void candidateCallback(const std_msgs::String::ConstPtr _msg){
    mbzirc::Candidate specs;
    specs.color = 1;
    hal::TaskState ts;

    std::stringstream msg;
    msg << _msg->data;
    mbzirc::CandidateList candidateList;
    msg >> candidateList;


    if(candidateList.candidates.size() > 0){
        mbzirc::Candidate target = bestCandidateMatch(candidateList, specs);
        std::cout << "tracking candidate with error " << target.location.transpose() << std::endl;
        target.location[2] = 15-gAltitude;
        pos_error_srv->send(target.location, ts);

    }else{
        hal::Vec3 zeroError = {0,0,0};
        pos_error_srv->send(zeroError, ts);
    }
    this_thread::sleep_for(chrono::milliseconds(100));
}

mbzirc::Candidate bestCandidateMatch(const mbzirc::CandidateList &_list, const mbzirc::Candidate &_specs){
    double bestScore= 0;
    mbzirc::Candidate bestCandidate;
    bestCandidate.location = {999,9999,9999};
    for(auto&candidate:_list.candidates){
        double score = 0;
        if(candidate.color == _specs.color){
            score +=1;
        }

        if(candidate.shape == _specs.shape){
            score +=1;
        }

        if((candidate.location - _specs.location).norm() < (bestCandidate.location - _specs.location).norm()){
            score +=1;
        }

        if(score > bestScore){
            bestCandidate = candidate;
        }
    }
    return bestCandidate;
}
