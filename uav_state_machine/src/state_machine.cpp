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
//----------
#include <uav_state_machine/state_machine.h>
#include <thread>

using namespace uav_state_machine;

UavStateMachine::UavStateMachine(grvc::utils::ArgumentParser _args) : HalClient(_args) {
    std::string uav_id = _args.getArgument<std::string>("uavId", "1");
    uav_id_ = atoi(uav_id.c_str());
    ros::NodeHandle nh;
    catching_device_   = new CatchingDevice(std::stoi(uav_id), nh);
    take_off_service_  = nh.advertiseService("/mbzirc_" + uav_id + "/uav_state_machine/takeoff",  &UavStateMachine::takeoffServiceCallback, this);
    land_service_      = nh.advertiseService("/mbzirc_" + uav_id + "/uav_state_machine/land",     &UavStateMachine::landServiceCallback, this);
    search_service_    = nh.advertiseService("/mbzirc_" + uav_id + "/uav_state_machine/waypoint", &UavStateMachine::searchServiceCallback, this);
    target_service_    = nh.advertiseService("/mbzirc_" + uav_id + "/uav_state_machine/enabled",  &UavStateMachine::targetServiceCallback, this);

    position_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/mavros_" + uav_id + "/local_position/pose", 10, &UavStateMachine::positionCallback, this);
    altitude_sub_ = nh.subscribe<std_msgs::Float64>("/mavros_" + uav_id + "/global_position/rel_alt", 10, &UavStateMachine::altitudeCallback, this);
    lidar_altitude_sub_ = nh.subscribe<sensor_msgs::Range>("/mavros_" + uav_id + "/distance_sensor/lidarlite_pub", 10, &UavStateMachine::lidarAltitudeCallback, this);
    lidar_altitude_remapped_pub_ = nh.advertise<std_msgs::Float64>("/mbzirc_" + uav_id + "/uav_state_machine/lidar_altitude", 1);
    joy_sub_ = nh.subscribe<sensor_msgs::Joy>("/joy", 10, &UavStateMachine::joyCallback, this);

    // Matched candidate can't be invalid until first detection...
    matched_candidate_.header.stamp.sec = 0;  // ...so initialize its timestamp in epoch
    matched_candidate_.header.stamp.nsec = 0;

    // Initial state is repose
    state_.state = uav_state::REPOSE;
    state_publisher_ = nh.advertise<uav_state_machine::uav_state>("/mbzirc_" + uav_id + "/uav_state_machine/state", 1);
	    
    state_pub_thread_ = std::thread([&](){
        while(ros::ok()){	
            state_publisher_.publish(this->state_);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });
}

//-------------------------------------------------------------------------------------------------------------------------------
bool UavStateMachine::init() {
     // Wait for hal connection
    while(!take_off_srv_->isConnected()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "Initialized" << std::endl;
    std::cout << "Connected to hal" << std::endl;

    return true;
}

//--------------------------------------------------------------------------------------------------------------------------------
void UavStateMachine::step() {
    grvc::hal::TaskState ts;
    switch (state_.state) {

        case uav_state::REPOSE:
            break;

        case uav_state::TAKINGOFF:
            take_off_srv_->send(target_altitude_, ts);
            state_.state = uav_state::HOVER;
            break;

        case uav_state::HOVER:
            break;

        case uav_state::SEARCHING:
            onSearching();
            break;

        case uav_state::CATCHING:
            onCatching();
            break;

        case uav_state::GOTO_DEPLOY:
            onGoToDeploy();
            break;

        case uav_state::LANDING:
            land_srv_->send(ts);
            state_.state = uav_state::REPOSE;
            break;

        default:
            assert(false);  // Must be an error!
            break;
    }
}

//---------------------------------------------------------------------------------------------------------------------------------
void UavStateMachine::onSearching() {
    // When the UAV finish the track, it starts again the same track
    grvc::hal::TaskState ts;
    waypoint_srv_->send(waypoint_list_[waypoint_index_], ts);
    if (ts == grvc::hal::TaskState::finished) {
        waypoint_index_++;
        if (waypoint_index_ > (waypoint_list_.size()-1)) {
            waypoint_index_ = 0;
            state_.state = uav_state::HOVER;
        }
    }
}

//---------------------------------------------------------------------------------------------------------------------------------
void UavStateMachine::onCatching() {
    //GotoTargetPosition

    //TargetTracking (aka visual servoing)
    /// Init subscriber to candidates
    ros::NodeHandle nh;
    ros::Subscriber candidate_subscriber = nh.subscribe<uav_state_machine::candidate_list>("/mbzirc_" + std::to_string(uav_id_) +"/candidateList", 1, &UavStateMachine::candidateCallback, this);

    if (!candidate_subscriber) {
        std::cout << "Can't start candidate subscriber." << std::endl;
        state_.state = uav_state::HOVER;
	    return;
    } else {
        std::cout << "Subscribed to candidate topic" << std::endl;
    }

    while (state_.state == uav_state::CATCHING) {
        ros::Duration since_last_candidate = ros::Time::now() - matched_candidate_.header.stamp;
        ros::Duration timeout(1.0);  // TODO: from config, in [s]?
        if (since_last_candidate < timeout) {
            // x-y-control: in candidateCallback
            // z-control: descend
            target_position_[2] = -0.1;  // TODO: As a function of x-y error?
            //target_position_[2] = target_altitude_-current_altitude_;  // From joystick!
        } else {
            // x-y-control slowly goes to 0
            target_position_[0] = 0.9*target_position_[0];
            target_position_[1] = 0.9*target_position_[1];
            // z-control: ascend
            target_position_[2] = +0.1;  // TODO: As a function of x-y error?
            //target_position_[2] = target_altitude_-current_altitude_;  // From joystick!
        }
        // TODO: Check max altitude and change state to LOST?
        // Send target_position
        grvc::hal::TaskState ts;
        pos_error_srv_->send(target_position_, ts);

        if (catching_device_->switchIsPressed()) {
            state_.state = uav_state::GOTO_DEPLOY;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    //Pickup
    //GotoDeploy
}

//---------------------------------------------------------------------------------------------------------------------------------
void UavStateMachine::onGoToDeploy() {
    // Go up!
    grvc::hal::Waypoint up_waypoint = current_position_waypoint_;
    up_waypoint.pos.z() = 5.0;  // TODO: Altitude as a parameter
    grvc::hal::TaskState ts;
    waypoint_srv_->send(up_waypoint, ts);  // Blocking!
    // TODO: Go to deploy zone (what switch turns off?)
    state_.state = uav_state::HOVER;
}

//---------------------------------------------------------------------------------------------------------------------------------
bool UavStateMachine::takeoffServiceCallback(uav_state_machine::takeoff_service::Request &req, uav_state_machine::takeoff_service::Response &res) {
    if (state_.state == uav_state::REPOSE) {
        target_altitude_ = req.altitude;
        state_.state = uav_state::TAKINGOFF;
        res.success = true;
    } else {
        res.success = false;
    }
    return true;
}

//---------------------------------------------------------------------------------------------------------------------------------
bool UavStateMachine::landServiceCallback(uav_state_machine::land_service::Request &req, uav_state_machine::land_service::Response &res) {
    target_altitude_ = 0;
    if (state_.state != uav_state::REPOSE) {
        state_.state = uav_state::LANDING;
        res.success = true;
    } else {
        std::cout << "Already landed!" << std::endl;
        res.success = false;
    }
    return true;
}

//---------------------------------------------------------------------------------------------------------------------------------
bool UavStateMachine::searchServiceCallback(uav_state_machine::waypoint_service::Request &req, uav_state_machine::waypoint_service::Response &res) {
    // The difference between start and restart is that restart continues by the last wp
    switch (req.action) {

        case waypoint_serviceRequest::STOP:
            if (state_.state == uav_state::SEARCHING) {
                state_.state = uav_state::HOVER;
                grvc::hal::TaskState ts;
                abort_srv_->send(ts);
                waypoint_srv_->send(current_position_waypoint_, ts);
                res.success = true;
            } else {
                res.success = false;
            }
            break;

        case waypoint_serviceRequest::START:
            if (state_.state == uav_state::HOVER) {
                waypoint_index_ = 0;
                waypoint_list_.clear();
                for (int i=0; i< req.waypoint_track.size(); i++) {
                    waypoint_list_.push_back({{req.waypoint_track[i].x,
                    req.waypoint_track[i].y, req.waypoint_track[i].z}, 0.0});
                }
                state_.state = uav_state::SEARCHING;
                res.success = true;
            } else {
                res.success = false;
            }
            break;

        case waypoint_serviceRequest::RESTART:
            if (state_.state == uav_state::HOVER) {
                state_.state = uav_state::SEARCHING;
                res.success = true;
            } else {
                res.success = false;
            }
            break;

        default:
            assert(false);  // Must be an error!
            break;
    }
    return true;
    //TargetTracking
    //Pickup
    //GotoDeploy
}

//---------------------------------------------------------------------------------------------------------------------------------
bool UavStateMachine::targetServiceCallback(uav_state_machine::target_service::Request  &req,
         uav_state_machine::target_service::Response &res)
{
    std::cout << "Received target of color: " << req.color << ", and position :[" << req.position[0] << ", " << req.position[1] << "];" << std::endl;
    if (state_.state == uav_state::HOVER && req.enabled) {
        target_.color = req.color;
        target_.shape = req.shape;
        target_.position.x = req.position[0];
        target_.position.y = req.position[1];
        res.success = true;
        state_.state = uav_state::CATCHING;
        return true;
    } else if(state_.state == uav_state::CATCHING && !req.enabled){
        state_.state = uav_state::HOVER;
        return true;
    }else {
        res.success = false;
        return false;
    }
}

//---------------------------------------------------------------------------------------------------------------------------------
void UavStateMachine::altitudeCallback(const std_msgs::Float64::ConstPtr& _msg){
    current_altitude_ = _msg->data;
}
//---------------------------------------------------------------------------------------------------------------------------------
void UavStateMachine::lidarAltitudeCallback(const sensor_msgs::Range::ConstPtr& _msg){
    std_msgs::Float64 altitude;
    altitude.data = _msg->range;
    lidar_altitude_remapped_pub_.publish(altitude);
}

//---------------------------------------------------------------------------------------------------------------------------------
void UavStateMachine::positionCallback(const geometry_msgs::PoseStamped::ConstPtr& _msg){
    current_position_waypoint_ = {{_msg->pose.position.x, _msg->pose.position.y, _msg->pose.position.z}, 0.0};
}
//---------------------------------------------------------------------------------------------------------------------------------
void UavStateMachine::joyCallback(const sensor_msgs::Joy::ConstPtr& _joy) {
    target_altitude_ = current_altitude_ + _joy->axes[1];
    //std::cout << "Target altitude: " << target_altitude_ << std::endl;
}

//---------------------------------------------------------------------------------------------------------------------------------
void UavStateMachine::candidateCallback(const uav_state_machine::candidate_list::ConstPtr& _msg) {
    //mbzirc::Candidate specs;
    //specs.color = 1;
    if (state_.state == uav_state::CATCHING) {
        uav_state_machine::candidate_list candidateList = *_msg;
        if (candidateList.candidates.size() > 0) {
            if (bestCandidateMatch(candidateList, target_, matched_candidate_)) {
                matched_candidate_.header.stamp = ros::Time::now();
                std::cout << "tracking candidate with error " << matched_candidate_.position << std::endl;
                target_position_[0] = matched_candidate_.position.x;
                target_position_[1] = matched_candidate_.position.y;
            } else {
                std::cout << "Cant find a valid candidate" << std::endl;
            }
        } else {
            std::cout << "Candidate list is empty!" << std::endl;
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
bool UavStateMachine::bestCandidateMatch(const uav_state_machine::candidate_list _list, const uav_state_machine::candidate &_specs, uav_state_machine::candidate &_result) {
    double bestScore = 0;
    bool foundMatch = false;
    for (auto&candidate:_list.candidates) {
        double score = 0;

        if (candidate.color == _specs.color) {
            score +=1;
        }

        //if(candidate.shape == _specs.shape){
        //    score +=1;
        //}

        //if((candidate.location - _specs.location).norm() < (_result.location - _specs.location).norm()){
        //    score +=1;
        //}

        if (score > bestScore) {
            _result = candidate;
            foundMatch = true;
        }
    }
    return foundMatch;
}
