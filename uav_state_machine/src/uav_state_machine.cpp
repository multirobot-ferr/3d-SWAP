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
//----------
#include <uav_state_machine/uav_state_machine.h>
#include <sstream>
#include <thread>
#include <math.h>
#include <gcs_state_machine/ApproachPoint.h>
#include <gcs_state_machine/DeployArea.h>
#include <uav_state_machine/SwitchVision.h>

#define Z_GIVE_UP_CATCHING 15.0  // TODO: From config file?
#define Z_RETRY_CATCH 1.0
#define Z_STOP_FREE_FALLING 0.15

using namespace uav_state_machine;

UavStateMachine::UavStateMachine(grvc::utils::ArgumentParser _args) : ual_(_args) {
    flying_level_ = _args.getArgument<float>("flying_level", 10.0);
    std::string uav_id = _args.getArgument<std::string>("uav_id", "1");
    uav_id_ = atoi(uav_id.c_str());
    ros::NodeHandle nh;
    catching_device_   = CatchingDevice::createCatchingDevice(std::stoi(uav_id), nh);
    take_off_service_  = nh.advertiseService("mbzirc_" + uav_id + "/uav_state_machine/takeoff",  &UavStateMachine::takeoffServiceCallback, this);
    land_service_      = nh.advertiseService("mbzirc_" + uav_id + "/uav_state_machine/land",     &UavStateMachine::landServiceCallback, this);
    search_service_    = nh.advertiseService("mbzirc_" + uav_id + "/uav_state_machine/waypoint", &UavStateMachine::searchServiceCallback, this);
    target_service_    = nh.advertiseService("mbzirc_" + uav_id + "/uav_state_machine/enabled",  &UavStateMachine::targetServiceCallback, this);
    // @Capi
    // target_status_client_ = nh.serviceClient<mbzirc_scheduler::SetTargetStatus>("scheduler/set_target_status");
    deploy_approach_client_ = nh.serviceClient<gcs_state_machine::ApproachPoint>("mbzirc_gcs/approach_point");
    deploy_area_client_ = nh.serviceClient<gcs_state_machine::DeployArea>("mbzirc_gcs/deploy_area");
    vision_algorithm_switcher_client_ = nh.serviceClient<SwitchVision>("mbzirc_" + uav_id + "/vision_node/algorithm_service");

    lidar_altitude_sub_ = nh.subscribe<sensor_msgs::Range>("mavros_" + uav_id + "/distance_sensor/lidarlite_pub", 10, &UavStateMachine::lidarAltitudeCallback, this);
    lidar_altitude_remapped_pub_ = nh.advertise<std_msgs::Float64>("mbzirc_" + uav_id + "/uav_state_machine/lidar_altitude", 1);

    // Matched candidate can't be invalid until first detection...
    matched_candidate_.header.stamp.sec = 0;  // ...so initialize its timestamp in epoch
    matched_candidate_.header.stamp.nsec = 0;
    max_tries_counter_ = _args.getArgument<int>("max_tries_catching", 5);

    // Initial state is repose
    state_.state = UavState::REPOSE;
    state_publisher_ = nh.advertise<UavState>("mbzirc_" + uav_id + "/uav_state_machine/state", 1);

    state_pub_thread_ = std::thread([this]() {
        ros::Rate loop_rate(10);  // [Hz]
        while (ros::ok()) {
            switch(this->state_.state) {
                case UavState::REPOSE:
                this->state_.state_str.data = std::string("REPOSE");
                break;
                case UavState::TAKINGOFF:
                this->state_.state_str.data = std::string("TAKINGOFF");
                break;
                case UavState::HOVER:
                this->state_.state_str.data = std::string("HOVER");
                this->state_.state_msg.data = std::string("");
                break;
                case UavState::SEARCHING:
                this->state_.state_str.data = std::string("SEARCHING");
                break;
                case UavState::CATCHING:
                this->state_.state_str.data = std::string("CATCHING");
                break;
                case UavState::LANDING:
                this->state_.state_str.data = std::string("LANDING");
                break;
                case UavState::GOTO_DEPLOY:
                this->state_.state_str.data = std::string("GOTO_DEPLOY");
                break;
                case UavState::GOTO_CATCH:
                this->state_.state_str.data = std::string("GOTO_CATCH");
                this->state_.state_msg.data = std::to_string(this->target_.target_id);
                break;
                case UavState::RETRY_CATCH:
                this->state_.state_str.data = std::string("RETRY_CATCH");
                break;
                case UavState::ERROR:
                this->state_.state_str.data = std::string("ERROR");
                break;
            }	
            this->state_.uav_id = this->uav_id_;

            state_publisher_.publish(this->state_);
            loop_rate.sleep();
        }
    });
}


void UavStateMachine::init() {
     // Wait for ual initialization
    while (!ual_.isReady() && ros::ok()) {
        sleep(1);
    }
    ROS_INFO("UAV State Machine [%d] initialized!", uav_id_);
}


void UavStateMachine::step() {
    gcs_state_machine::ApproachPoint approach_call;
    switch (state_.state) {

        case UavState::REPOSE:
            break;

        case UavState::TAKINGOFF:
            ual_.takeOff(target_altitude_);
            hover_position_waypoint_ = ual_.pose();
            state_.state = UavState::SEARCHING;
            break;

        case UavState::HOVER:
            ual_.goToWaypoint(hover_position_waypoint_);
            //std::this_thread::sleep_for(std::chrono::milliseconds(500));  // TODO: Needed?
            break;

        case UavState::SEARCHING:
            onSearching();
            break;

        case UavState::GOTO_CATCH:
        {
            // Enable candidate search vision algorithm
            SwitchVision switchRequest;
            switchRequest.request.algorithm =  SwitchVision::Request::ALGORITHM_CANDIDATES;
            if (!vision_algorithm_switcher_client_.call(switchRequest)) {
                state_.state = UavState::ERROR;
                state_.state_str.data = "Error enabling candidate detector algorithm";
                return;
            }
            // Free approach point
            approach_call.request.uav_id = uav_id_;
            approach_call.request.question = gcs_state_machine::ApproachPoint::Request::FREE_APPROACH_POINT;
            deploy_approach_client_.call(approach_call);
            if (approach_call.response.answer != gcs_state_machine::ApproachPoint::Response::OK) {
                ROS_ERROR("Couldn't free approach point");
            }
            ual_.goToWaypoint(ual_.pose());
            grvc::ual::Waypoint target;
            target.pose.position.x = target_.global_position.x;
            target.pose.position.y = target_.global_position.y;
            target.pose.position.z = flying_level_;
            target.header.frame_id = "map";
            if (ual_.goToWaypoint(target)) {
                state_.state = UavState::CATCHING;
            } else {
                hover_position_waypoint_ = ual_.pose();
                state_.state = UavState::HOVER;
            }
            break;
        }
        case UavState::CATCHING:
            onCatching();
            break;

        case UavState::RETRY_CATCH:
        {
            grvc::ual::Waypoint target;
            target.pose.position.x = target_.global_position.x;
            target.pose.position.y = target_.global_position.y;
            target.pose.position.z = Z_RETRY_CATCH;
            target.header.frame_id = "map";
            if (ual_.goToWaypoint(target)) {
                state_.state = UavState::CATCHING;
            } else {
                hover_position_waypoint_ = ual_.pose();
                state_.state = UavState::HOVER;
            }
            break;
        }
        case UavState::GOTO_DEPLOY:
            onGoToDeploy();
            break;

        case UavState::LANDING:
            ual_.land();
            state_.state = UavState::REPOSE;
            break;
        case UavState::ERROR:
            break;
        default:
            assert(false);  // Must be an error!
            break;
    }
}


void UavStateMachine::onSearching() {
    // Enable candidate search vision algorithm
    SwitchVision switchRequest;
    switchRequest.request.algorithm = SwitchVision::Request::ALGORITHM_CANDIDATES;
    if (!vision_algorithm_switcher_client_.call(switchRequest)) {
        state_.state = UavState::ERROR;
        state_.state_str.data = "Error enabling candidate detector algorithm";
        return;
    }

    // When the UAV finish the track, it starts again the same track
    if (ual_.goToWaypoint(waypoint_list_[waypoint_index_])) {
        waypoint_index_++;
        if (waypoint_index_ > (waypoint_list_.size()-1)) {
            waypoint_index_ = 0;
            hover_position_waypoint_ = ual_.pose();
            state_.state = UavState::HOVER;
        }
    }
}


void UavStateMachine::onCatching() {
    // TargetTracking (aka visual servoing)
    // Init subscriber to candidates
    ros::NodeHandle nh;
    ros::Subscriber candidate_subscriber = nh.subscribe<CandidateList>("mbzirc_" + std::to_string(uav_id_) + \
        "/candidateList", 1, &UavStateMachine::candidateCallback, this);

    if (!candidate_subscriber) {
        ROS_WARN("Can't start candidate subscriber.");
        hover_position_waypoint_ = ual_.pose();
        state_.state = UavState::HOVER;
	    return;
    } else {
        ROS_INFO("Subscribed to candidate topic");
    }

    // Magnetize catching device
    catching_device_->setMagnetization(true);

    unsigned tries_counter = 0;

    std::vector<double> history_xy_errors;

    bool free_fall = false;
    ros::Rate loop_rate(10);  // [Hz]
    while (state_.state == UavState::CATCHING) {
        ros::Duration since_last_candidate = ros::Time::now() - matched_candidate_.header.stamp;
        ros::Duration timeout(1.0);  // TODO: from config, in [s]?

        if (current_altitude_ < Z_STOP_FREE_FALLING) {
            free_fall = false;
            target_position_[2] = 0.0;  // TODO: Go to retry alttitude here?
        }

        // TODO: Refactor free-fall!!!
        if (since_last_candidate < timeout) {
            // x-y-control: in candidateCallback
            // z-control: descend
            if (current_altitude_ < 1.0) {
                double xy_error = sqrt(target_position_[0]*target_position_[0] + target_position_[1]*target_position_[1]);
                history_xy_errors.push_back(xy_error);
                
                if(history_xy_errors.size() > 10){   // 666 TODO: Check window size for avg xy error!
                    history_xy_errors.erase(history_xy_errors.begin());

                    double avg_xy_error;
                    avg_xy_error = std::accumulate(history_xy_errors.begin(), history_xy_errors.end(), 0);
                    avg_xy_error /= history_xy_errors.size();
                    
                    if (avg_xy_error < 0.1) {
                        target_position_[2] = -0.22;  // TODO: As a function of x-y error?
                        free_fall = true;
                    } else if (!free_fall) {
                            target_position_[2] = 1.0-current_altitude_;  // Hold at 1m
                    } else {
                            target_position_[2] = -0.22;
                    }
                } else {
                    target_position_[2] = 1.0-current_altitude_;  // Hold at 1m
                }
            } else {
                target_position_[2] = -0.5;  // TODO: As a function of x-y error?
            }
        } else {   // No fresh candidates (timeout)
	        if (!free_fall) {
                if(current_altitude_ > flying_level_*0.75){
	        	    target_position_[2] = +1.0;  // TODO: As a function of x-y error?
                } else {
                    // Go up in the same position.
                    grvc::ual::Waypoint up_waypoint = ual_.pose();
                    up_waypoint.pose.position.z = 2.0;  // TODO: Altitude as a parameter
                    ual_.goToWaypoint(up_waypoint);  // Blocking!

                    // Move to target position.
                    grvc::ual::Waypoint approaching_waypoint = ual_.pose();
                    approaching_waypoint.pose.position.x = target_.global_position.x;
                    approaching_waypoint.pose.position.y = target_.global_position.y;
                    approaching_waypoint.pose.position.z = 1.0;
                    approaching_waypoint.header.frame_id = "map";
                    ual_.goToWaypoint(approaching_waypoint);  // Blocking!

                    tries_counter++;
                    if (tries_counter > max_tries_counter_) {
                        // Go to initial catch position.
                        grvc::ual::Waypoint initial_catch = ual_.pose();
                        initial_catch.pose.position.x = target_.global_position.x;
                        initial_catch.pose.position.y = target_.global_position.y;
                        initial_catch.pose.position.z = flying_level_;
                        initial_catch.header.frame_id = "map";                    
                        ual_.goToWaypoint(initial_catch);

                        // @Capi
                        // Set target to failed
                        // mbzirc_scheduler::SetTargetStatus target_status_call;
                        // target_status_call.request.target_id = target_.target_id;
                        // target_status_call.request.target_status = mbzirc_scheduler::SetTargetStatus::Request::FAILED;
                        // if (!target_status_client_.call(target_status_call)) {
                        //     ROS_ERROR("Error setting target status to FAILED in UAV_%d", uav_id_);
                        // }                                           

                        // Switch to HOVER state.
                        hover_position_waypoint_ = ual_.pose();
                        state_.state = UavState::HOVER;
                        // Break loop.
                        return;
                    }
                }
            } else {
                target_position_[2] = -0.22;
		    }
	    }
        // Send target_position
        grvc::ual::PositionError error;  // TODO: build function?
        error.vector.x = target_position_[0];
        error.vector.y = target_position_[1];
        error.vector.z = target_position_[2];
        ual_.setPositionError(error);

        if (catching_device_->switchIsPressed()) {
            state_.state = UavState::GOTO_DEPLOY;
        }

        // If we're too high, give up
        if (current_altitude_ > Z_GIVE_UP_CATCHING) {
            // @Capi
            // mbzirc_scheduler::SetTargetStatus target_status_call;
            // target_status_call.request.target_id = target_.target_id;
            // target_status_call.request.target_status = mbzirc_scheduler::SetTargetStatus::Request::LOST;
            // if (!target_status_client_.call(target_status_call)) {
            //     ROS_ERROR("Error setting target status to LOST in UAV_%d", uav_id_);
            // }
            hover_position_waypoint_ = ual_.pose();
            state_.state = UavState::HOVER;
        }

        // TODO: Review this frequency!
        loop_rate.sleep();
    }
}


void UavStateMachine::onGoToDeploy() {
    // Disable searching
    SwitchVision switchRequest;
    switchRequest.request.algorithm =  SwitchVision::Request::ALGORITHM_DISABLE;
    if (!vision_algorithm_switcher_client_.call(switchRequest)) {
        state_.state = UavState::ERROR;
        state_.state_str.data = "Error disabling vision algorithm";
        return;
    }
    // Go up!
    grvc::ual::Waypoint up_waypoint = ual_.pose();
    up_waypoint.pose.position.z = 5.0;  // TODO: Altitude as a parameter
    ual_.goToWaypoint(up_waypoint);  // Blocking!
    // TODO: Go to deploy zone (what if switch turns off?)
    if (catching_device_->switchIsPressed()) {  // Check switch again
        // Update target status to CAUGHT
        // @Capi
        // mbzirc_scheduler::SetTargetStatus target_status_call;
		// target_status_call.request.target_id = target_.target_id;
        // target_status_call.request.target_status = mbzirc_scheduler::SetTargetStatus::Request::CAUGHT;
		// if (!target_status_client_.call(target_status_call)) {
		//     ROS_ERROR("Error setting target status to CAUGHT in UAV_%d", uav_id_);
		// }
        // Go to closest deploy point
        gcs_state_machine::ApproachPoint approach_call;
        approach_call.request.uav_id = uav_id_;
        approach_call.request.uav_position.x = ual_.pose().pose.position.x; // @Arturo TODO: enviar en coordenadas map
        approach_call.request.uav_position.y = ual_.pose().pose.position.y;
        approach_call.request.question = gcs_state_machine::ApproachPoint::Request::RESERVE_APPROACH_POINT;
        deploy_approach_client_.call(approach_call);
        grvc::ual::Waypoint approach_waypoint;
        if (approach_call.response.answer == gcs_state_machine::ApproachPoint::Response::OK) {
            approach_waypoint.header.frame_id = "map";
            approach_waypoint.pose.position.x = approach_call.response.approach_position.x;
            approach_waypoint.pose.position.y = approach_call.response.approach_position.y;
            approach_waypoint.pose.position.z = flying_level_;
            ual_.goToWaypoint(approach_waypoint);  // Blocking!
        } else {
            ROS_ERROR("Must be an error, not available approach points!");
            return;
        }
        // TODO: Check dropping zone is free
        gcs_state_machine::DeployArea deploy_call;
        deploy_call.request.uav_id = uav_id_;
        deploy_call.request.question = gcs_state_machine::DeployArea::Request::RESERVE_DEPLOY_AREA;
        do {
            deploy_area_client_.call(deploy_call);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        } while (deploy_call.response.answer == gcs_state_machine::DeployArea::Response::WAIT);
        grvc::ual::Waypoint deploy_waypoint;
        deploy_waypoint.header.frame_id = "map";
        deploy_waypoint.pose.position.x = deploy_call.response.deploy_position.x;
        deploy_waypoint.pose.position.y = deploy_call.response.deploy_position.y;
        deploy_waypoint.pose.position.z = flying_level_;
        ual_.goToWaypoint(deploy_waypoint);  // Blocking!
        grvc::ual::Waypoint down_waypoint = deploy_waypoint;
        down_waypoint.pose.position.z = 3.0;  // TODO: Altitude as a parameter
        ual_.goToWaypoint(down_waypoint);  // Blocking!
        // Demagnetize catching device
        catching_device_->setMagnetization(false);
        // TODO Check !catching_device_->switchIsPressed()
        // Update target status to DEPLOYED
        // @Capi
        // target_status_call.request.target_status = mbzirc_scheduler::SetTargetStatus::Request::DEPLOYED;
		// if (!target_status_client_.call(target_status_call)) {
		//     ROS_ERROR("Error setting target status to DEPLOYED in UAV_%d", uav_id_);
		// }
        up_waypoint = ual_.pose();
        up_waypoint.pose.position.z = flying_level_;
        ual_.goToWaypoint(up_waypoint);  // Blocking!
        ual_.goToWaypoint(approach_waypoint);  // Blocking!
        deploy_call.request.question = gcs_state_machine::DeployArea::Request::FREE_DEPLOY_AREA;
        do {
            deploy_area_client_.call(deploy_call);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        } while (deploy_call.response.answer == gcs_state_machine::DeployArea::Response::WAIT);
        hover_position_waypoint_ = ual_.pose();
        state_.state = UavState::HOVER;
    } else {
        ROS_INFO("Miss the catch, try again!");
        state_.state = UavState::CATCHING;
    }
}


bool UavStateMachine::takeoffServiceCallback(TakeOff::Request &req, TakeOff::Response &res) {
    if (state_.state == UavState::REPOSE) {
        target_altitude_ = req.altitude;
        state_.state = UavState::TAKINGOFF;
        res.success = true;
    } else {
        res.success = false;
    }
    return true;
}


bool UavStateMachine::landServiceCallback(Land::Request &req, Land::Response &res) {
    target_altitude_ = 0;
    if (state_.state != UavState::REPOSE) {
        state_.state = UavState::LANDING;
        res.success = true;
    } else {
        std::cout << "Already landed!" << std::endl;
        res.success = false;
    }
    return true;
}


bool UavStateMachine::searchServiceCallback(TrackPath::Request &req, TrackPath::Response &res) {
    // The difference between start and restart is that restart continues by the last wp
    waypoint_index_ = 0;
    waypoint_list_.clear();
    for (int i=0; i< req.waypoint_track.size(); i++) {
        grvc::ual::Waypoint wp;
        wp.header.frame_id = req.waypoint_track[i].header.frame_id;
        wp.pose.position = req.waypoint_track[i].pose.position;
        wp.pose.orientation.w = 1;
        waypoint_list_.push_back(wp);
    }
    res.success = true;
     
    return true;
}


bool UavStateMachine::targetServiceCallback(SetTarget::Request  &req,
         SetTarget::Response &res)
{
    ROS_INFO("Received target of color: %d, and position [%f, %f]", \
        req.color, req.global_position.x, req.global_position.y);
    if (state_.state == UavState::HOVER && req.enabled) {
        target_ = req;
        res.success = true;
        state_.state = UavState::GOTO_CATCH;
        return true;
    } else if (state_.state == UavState::CATCHING && !req.enabled){
        hover_position_waypoint_ = ual_.pose();
        state_.state = UavState::HOVER;
        return true;
    } else {
        res.success = false;
        return false;
    }
}


void UavStateMachine::lidarAltitudeCallback(const sensor_msgs::Range::ConstPtr& _msg){
    current_altitude_ = _msg->range;
    std_msgs::Float64 altitude;
    altitude.data = current_altitude_;
    lidar_altitude_remapped_pub_.publish(altitude);
}


void UavStateMachine::candidateCallback(const CandidateList::ConstPtr& _msg) {
    if (state_.state == UavState::CATCHING) {
        CandidateList candidateList = *_msg;
        if (candidateList.candidates.size() > 0) {
            Candidate target_candidate;
            target_candidate.color = target_.color;
            target_candidate.shape = target_.shape;
            target_candidate.global_position.x = target_.global_position.x;
            target_candidate.global_position.y = target_.global_position.y;
            if (bestCandidateMatch(candidateList, target_candidate, matched_candidate_)) {
                matched_candidate_.header.stamp = ros::Time::now();
                target_position_[0] = matched_candidate_.local_position.x;
                target_position_[1] = matched_candidate_.local_position.y;
            }
        } else {
            std::cout << "Candidate list is empty!" << std::endl;
        }
    }
}


bool UavStateMachine::bestCandidateMatch(const CandidateList _list, const Candidate &_specs, Candidate &_result) {
    double bestScore = 0;
    bool foundMatch = false;
    
    typedef std::pair<double, Candidate> PairDistanceCandidate;
    std::vector<PairDistanceCandidate> pairsDistCands;

    Eigen::Vector3d specPos = {_specs.global_position.x, _specs.global_position.y, _specs.global_position.z};
    // @Arturo
    /*for (auto&candidate:_list.candidates) {
	bool isInField = frame_transform_.isInGameField( grvc::utils::constructPoint(   candidate.global_position.x,
                                                                                            candidate.global_position.y,
                                                                                            candidate.global_position.z) );

    bool isInDroppingArea = frame_transform_.isInDroppingArea( grvc::utils::constructPoint(   candidate.global_position.x,
                                                                                            candidate.global_position.y,
                                                                                            candidate.global_position.z) );

	if(isInField && !isInDroppingArea){
		Eigen::Vector3d candidatePos = {candidate.global_position.x, candidate.global_position.y, candidate.global_position.z};
		double dist = (specPos - candidatePos).norm();
		pairsDistCands.push_back(PairDistanceCandidate(dist, candidate));
	}
    }*/
    
    std::sort(pairsDistCands.begin(), pairsDistCands.end(), [](const PairDistanceCandidate &a, const PairDistanceCandidate &b) {
        return a.first < b.first;   
    });
        
    for(auto &pair: pairsDistCands){
        if (pair.second.color == _specs.color) {
            _result = pair.second;
            return true;
        }
    }
    return false;
}
