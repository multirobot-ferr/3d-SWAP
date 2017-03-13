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
#include <sstream>
#include <thread>
#include <math.h>
#include <gcs_state_machine/ApproachPoint.h>
#include <gcs_state_machine/DeployArea.h>
#include <grvc_utils/frame_transform.h>
#include <uav_state_machine/switch_vision.h>


#define Z_GIVE_UP_CATCHING 15.0  // TODO: From config file?
#define Z_RETRY_CATCH 1.0

using namespace uav_state_machine;

UavStateMachine::UavStateMachine(grvc::utils::ArgumentParser _args) : HalClient(_args) {
    flying_level_ = _args.getArgument<float>("flying_level", 10.0);
    std::string uav_id = _args.getArgument<std::string>("uavId", "1");
    uav_id_ = atoi(uav_id.c_str());
    ros::NodeHandle nh;
    catching_device_   = CatchingDevice::createCatchingDevice(std::stoi(uav_id), nh);
    take_off_service_  = nh.advertiseService("/mbzirc_" + uav_id + "/uav_state_machine/takeoff",  &UavStateMachine::takeoffServiceCallback, this);
    land_service_      = nh.advertiseService("/mbzirc_" + uav_id + "/uav_state_machine/land",     &UavStateMachine::landServiceCallback, this);
    search_service_    = nh.advertiseService("/mbzirc_" + uav_id + "/uav_state_machine/waypoint", &UavStateMachine::searchServiceCallback, this);
    target_service_    = nh.advertiseService("/mbzirc_" + uav_id + "/uav_state_machine/enabled",  &UavStateMachine::targetServiceCallback, this);
    target_status_client_ = nh.serviceClient<mbzirc_scheduler::SetTargetStatus>("/scheduler/set_target_status");
    deploy_approach_client_ = nh.serviceClient<gcs_state_machine::ApproachPoint>("/gcs/approach_point");
    deploy_area_client_ = nh.serviceClient<gcs_state_machine::DeployArea>("/gcs/deploy_area");
    vision_algorithm_switcher_client_ = nh.serviceClient<uav_state_machine::switch_vision>("/mbzirc_" + uav_id + "/vision_node/algorithm_service");

    //position_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/mavros_" + uav_id + "/local_position/pose", 10, &UavStateMachine::positionCallback, this);
	position_sub_ = nh.subscribe<std_msgs::String>("/mbzirc_" + uav_id + "/hal/pose", 10, &UavStateMachine::positionCallback, this);
    altitude_sub_ = nh.subscribe<std_msgs::Float64>("/mavros_" + uav_id + "/global_position/rel_alt", 10, &UavStateMachine::altitudeCallback, this);
    lidar_altitude_sub_ = nh.subscribe<sensor_msgs::Range>("/mavros_" + uav_id + "/distance_sensor/lidarlite_pub", 10, &UavStateMachine::lidarAltitudeCallback, this);
    lidar_altitude_remapped_pub_ = nh.advertise<std_msgs::Float64>("/mbzirc_" + uav_id + "/uav_state_machine/lidar_altitude", 1);
    joy_sub_ = nh.subscribe<sensor_msgs::Joy>("/joy", 10, &UavStateMachine::joyCallback, this);

    // Matched candidate can't be invalid until first detection...
    matched_candidate_.header.stamp.sec = 0;  // ...so initialize its timestamp in epoch
    matched_candidate_.header.stamp.nsec = 0;
    max_tries_counter_ = _args.getArgument<int>("max_tries_catching", 3);

    // Initial state is repose
    state_.state = uav_state::REPOSE;
    state_publisher_ = nh.advertise<uav_state_machine::uav_state>("/mbzirc_" + uav_id + "/uav_state_machine/state", 1);
	    
    state_pub_thread_ = std::thread([&](){
        while(ros::ok()){

            switch(this->state_.state)
            {
                case uav_state::REPOSE:
                this->state_.state_str.data = std::string("REPOSE");
                break;
                case uav_state::TAKINGOFF:
                this->state_.state_str.data = std::string("TAKINGOFF");
                break;
                case uav_state::HOVER:
                this->state_.state_str.data = std::string("HOVER");
                this->state_.state_msg.data = std::string("");
                break;
                case uav_state::SEARCHING:
                this->state_.state_str.data = std::string("SEARCHING");
                break;
                case uav_state::CATCHING:
                this->state_.state_str.data = std::string("CATCHING");
                break;
                case uav_state::LANDING:
                this->state_.state_str.data = std::string("LANDING");
                break;
                case uav_state::GOTO_DEPLOY:
                this->state_.state_str.data = std::string("GOTO_DEPLOY");
                break;
                case uav_state::GOTO_CATCH:
                this->state_.state_str.data = std::string("GOTO_CATCH");
                this->state_.state_msg.data = std::to_string(target_.target_id);
                break;
                case uav_state::RETRY_CATCH:
                this->state_.state_str.data = std::string("RETRY_CATCH");
                break;
                case uav_state::ERROR:
                this->state_.state_str.data = std::string("ERROR");
                break;
            }	
            this->state_.uav_id = uav_id_;

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
/*
    grvc::utils::frame_transform frameTransform;
    geometry_msgs::Point deploy_point = frameTransform.game2map(grvc::utils::constructPoint(5.6,20.7,3.0));
    deploy_waypoint_.pos.x() = deploy_point.x;
    deploy_waypoint_.pos.y() = deploy_point.y;
    deploy_waypoint_.pos.z() = flying_level_;
    //deploy_waypoint_.yaw = current_position_waypoint_.yaw;
*/
    return true;
}

//--------------------------------------------------------------------------------------------------------------------------------
void UavStateMachine::step() {
    grvc::hal::TaskState ts;
    gcs_state_machine::ApproachPoint approach_call;
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

        case uav_state::GOTO_CATCH:
            approach_call.request.uav_id = uav_id_;
            approach_call.request.question = gcs_state_machine::ApproachPoint::Request::FREE_APPROACH_POINT;
            deploy_approach_client_.call(approach_call);
            if (approach_call.response.answer != gcs_state_machine::ApproachPoint::Response::OK) {
                std::cerr << "Couldn't free approach point" << std::endl;
            }
            waypoint_srv_->send({{current_position_waypoint_.pos.x(),
                                  current_position_waypoint_.pos.y(),
                                  flying_level_}, 0.0}, ts);
            waypoint_srv_->send({{target_.global_position.x,
                                  target_.global_position.y,
                                  flying_level_}, 0.0}, ts);
            if (ts == grvc::hal::TaskState::finished) {
                state_.state = uav_state::CATCHING;
            } else {
                state_.state = uav_state::HOVER;
            }
            break;

        case uav_state::CATCHING:
            onCatching();
            break;

        case uav_state::RETRY_CATCH:
            waypoint_srv_->send({{target_.global_position.x,
                                  target_.global_position.y,
                                  Z_RETRY_CATCH}, 0.0}, ts);
            if (ts == grvc::hal::TaskState::finished) {
                state_.state = uav_state::CATCHING;
            } else {
                state_.state = uav_state::HOVER;
            }
            break;

        case uav_state::GOTO_DEPLOY:
            onGoToDeploy();
            break;

        case uav_state::LANDING:
            land_srv_->send(ts);
            state_.state = uav_state::REPOSE;
            break;
        case uav_state::ERROR:
            break;
        default:
            assert(false);  // Must be an error!
            break;
    }
}

//---------------------------------------------------------------------------------------------------------------------------------
void UavStateMachine::onSearching() {
    // Enable candidate search vision algorithm
    uav_state_machine::switch_vision switchRequest;
    switchRequest.request.algorithm = uav_state_machine::switch_vision::Request::ALGORITHM_CANDIDATES;
    if (!vision_algorithm_switcher_client_.call(switchRequest)) {
        state_.state = uav_state::ERROR;
        state_.state_str.data = "Error enabling candidate detector algorithm";
        return;
    }

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
    // Enable candidate search vision algorithm
    uav_state_machine::switch_vision switchRequest;
    switchRequest.request.algorithm =  uav_state_machine::switch_vision::Request::ALGORITHM_CANDIDATES;
    if (!vision_algorithm_switcher_client_.call(switchRequest)) {
        state_.state = uav_state::ERROR;
        state_.state_str.data = "Error enabling candidate detector algorithm";
        return;
    }

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

    // Magnetize catching device
    catching_device_->setMagnetization(true);

    unsigned triesCounter = 0;

    std::vector<double> history_xy_errors;

    bool free_fall = false;
    while (state_.state == uav_state::CATCHING) {
        ros::Duration since_last_candidate = ros::Time::now() - matched_candidate_.header.stamp;
        ros::Duration timeout(1.0);  // TODO: from config, in [s]?

        if (current_altitude_ < 0.15) {
            free_fall = false;
            target_position_[2] = 0.0;  // TODO: Go to retry alttitude here?
        }

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
                    
                    if (avg_xy_error < 0.1) {
                        target_position_[2] = -0.22;  // TODO: As a function of x-y error?
                        free_fall = true;
                    } else if (!free_fall) {
                            target_position_[2] = 1.0-current_altitude_;  // Hold at 1m
                    } else {
                            target_position_[2] = -0.22;
                    }
                }else {
                    target_position_[2] = 1.0-current_altitude_;  // Hold at 1m
                }
            } else {
                target_position_[2] = -0.5;  // TODO: As a function of x-y error?
            }
        } else {   // No fresh candidates (timeout)
	        if (!free_fall) {
                std::cout << "Last candidate received " << since_last_candidate.toSec() << "s ago, ascend!" << std::endl;
                // Go up in the same position.
                grvc::hal::Waypoint up_waypoint = current_position_waypoint_;
                up_waypoint.pos.z() = 2.0;  // TODO: Altitude as a parameter
                grvc::hal::TaskState ts;
                waypoint_srv_->send(up_waypoint, ts);  // Blocking!

                // Move to target position.
                grvc::hal::Waypoint approachingWaypoint = {{target_.global_position.x, target_.global_position.y, 1.0}, current_position_waypoint_.yaw};
                waypoint_srv_->send(approachingWaypoint, ts);  // Blocking!
                
                triesCounter++;
                if(triesCounter > max_tries_counter_){
                    // Go to initial catch position.
                    waypoint_srv_->send({{ target_.global_position.x,
                                           target_.global_position.y,
                                           flying_level_}, 0.0}, ts);

                    // Switch to HOVER state.
                    state_.state = uav_state::HOVER;
                    // Break loop.
                    return;
                }
            } else {
                target_position_[2] = -0.22;
		    }
	    }
        // Send target_position
        grvc::hal::TaskState ts;
        pos_error_srv_->send(target_position_, ts);

        if (catching_device_->switchIsPressed()) {
            state_.state = uav_state::GOTO_DEPLOY;
        }

        // If we're too high, give up
        if (current_altitude_ > Z_GIVE_UP_CATCHING) {
            mbzirc_scheduler::SetTargetStatus target_status_call;
            target_status_call.request.target_id = target_.target_id;
            target_status_call.request.target_status = mbzirc_scheduler::SetTargetStatus::Request::LOST;
            if (!target_status_client_.call(target_status_call)) {
                ROS_ERROR("Error setting target status to LOST in UAV_%d", uav_id_);
            }
            state_.state = uav_state::HOVER;
        }

        // TODO: Review this frequency!
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

//---------------------------------------------------------------------------------------------------------------------------------
void UavStateMachine::onGoToDeploy() {
    // Disable searching
    uav_state_machine::switch_vision switchRequest;
    switchRequest.request.algorithm =  uav_state_machine::switch_vision::Request::ALGORITHM_DISABLE;
    if (!vision_algorithm_switcher_client_.call(switchRequest)) {
        state_.state = uav_state::ERROR;
        state_.state_str.data = "Error disabling vision algorithm";
        return;
    }

    // Go up!
    grvc::hal::Waypoint up_waypoint = current_position_waypoint_;
    up_waypoint.pos.z() = 5.0;  // TODO: Altitude as a parameter
    grvc::hal::TaskState ts;
    waypoint_srv_->send(up_waypoint, ts);  // Blocking!
    // TODO: Go to deploy zone (what if switch turns off?)
    if (catching_device_->switchIsPressed()) {  // Check switch again
        // Update target status to CAUGHT
        mbzirc_scheduler::SetTargetStatus target_status_call;
		target_status_call.request.target_id = target_.target_id;
        target_status_call.request.target_status = mbzirc_scheduler::SetTargetStatus::Request::CAUGHT;
		if (!target_status_client_.call(target_status_call)) {
		    ROS_ERROR("Error setting target status to CAUGHT in UAV_%d", uav_id_);
		}
        // Go to closest deploy point
        gcs_state_machine::ApproachPoint approach_call;
        approach_call.request.uav_id = uav_id_;
        approach_call.request.uav_position.x = current_position_waypoint_.pos.x();
        approach_call.request.uav_position.y = current_position_waypoint_.pos.y();
        approach_call.request.question = gcs_state_machine::ApproachPoint::Request::RESERVE_APPROACH_POINT;
        deploy_approach_client_.call(approach_call);
        grvc::hal::Waypoint approach_waypoint;
        if (approach_call.response.answer == gcs_state_machine::ApproachPoint::Response::OK) {
            approach_waypoint.pos.x() = approach_call.response.approach_position.x;
            approach_waypoint.pos.y() = approach_call.response.approach_position.y;
            approach_waypoint.pos.z() = flying_level_;
            waypoint_srv_->send(approach_waypoint, ts);  // Blocking!
        } else {
            std::cerr << "Must be an error, not available approach points!" << std::endl;
            return;
        }
        // TODO: Check dropping zone is free
        gcs_state_machine::DeployArea deploy_call;
        deploy_call.request.uav_id = uav_id_;
        deploy_call.request.question = gcs_state_machine::DeployArea::Request::RESERVE_DEPLOY_AREA;
        do {
            deploy_area_client_.call(deploy_call);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        } while(deploy_call.response.answer == gcs_state_machine::DeployArea::Response::WAIT);
        grvc::hal::Waypoint deploy_waypoint;
        deploy_waypoint.pos.x() = deploy_call.response.deploy_position.x;
        deploy_waypoint.pos.y() = deploy_call.response.deploy_position.y;
        deploy_waypoint.pos.z() = flying_level_;
        waypoint_srv_->send(deploy_waypoint, ts);  // Blocking!
        grvc::hal::Waypoint down_waypoint = deploy_waypoint;
        down_waypoint.pos.z() = 3.0;  // TODO: Altitude as a parameter
        waypoint_srv_->send(down_waypoint, ts);  // Blocking!
        // Demagnetize catching device
        catching_device_->setMagnetization(false);
        // TODO Check !catching_device_->switchIsPressed()
        // Update target status to DEPLOYED
        target_status_call.request.target_status = mbzirc_scheduler::SetTargetStatus::Request::DEPLOYED;
		if (!target_status_client_.call(target_status_call)) {
		    ROS_ERROR("Error setting target status to DEPLOYED in UAV_%d", uav_id_);
		}
        up_waypoint = current_position_waypoint_;
        up_waypoint.pos.z() = flying_level_;
        waypoint_srv_->send(up_waypoint, ts);  // Blocking!
        waypoint_srv_->send(approach_waypoint, ts);  // Blocking!
        deploy_call.request.question = gcs_state_machine::DeployArea::Request::FREE_DEPLOY_AREA;
        do {
            deploy_area_client_.call(deploy_call);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        } while(deploy_call.response.answer == gcs_state_machine::DeployArea::Response::WAIT);
        state_.state = uav_state::HOVER;
    } else {
        std::cout << "Miss the catch, try again!" << std::endl;
        state_.state = uav_state::CATCHING;
    }
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
    std::cout << "Received target of color: " << req.color << ", and position :[" \
    << req.global_position.x << ", " << req.global_position.y << "];" << std::endl;
    if (state_.state == uav_state::HOVER && req.enabled) {
        target_ = req;
        res.success = true;
        //state_.state = uav_state::CATCHING;
        state_.state = uav_state::GOTO_CATCH;
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
    //current_altitude_ = _msg->data;
    // Now current_altitude_ comes from lidar
}
//---------------------------------------------------------------------------------------------------------------------------------
void UavStateMachine::lidarAltitudeCallback(const sensor_msgs::Range::ConstPtr& _msg){
    const float object_height = 0.2;  // TODO: Check
    float delta_range = _msg->range - lidar_range_;
    if (delta_range > object_height) {
        lidar_reading_ = LidarReading::FLOOR;
    } else if (delta_range < -object_height) {
        lidar_reading_ = LidarReading::OBJECT;
    }
    lidar_range_ = _msg->range;  // Update anyway
    switch (lidar_reading_) {
        case LidarReading::FLOOR:
            current_altitude_ = lidar_range_;
            break;

        case LidarReading::OBJECT:
            current_altitude_ = lidar_range_ + object_height;
            break;

        default:
            current_altitude_ = lidar_range_;
            // But it's somekind of error!
            break;
    }
    std_msgs::Float64 altitude;
    altitude.data = current_altitude_;
    lidar_altitude_remapped_pub_.publish(altitude);
}

//---------------------------------------------------------------------------------------------------------------------------------
/*void UavStateMachine::positionCallback(const geometry_msgs::PoseStamped::ConstPtr& _msg) {
    // TODO: Make an util for quaternion to euler conversion? Eigen?
    double yaw = 2*atan2(_msg->pose.orientation.z, _msg->pose.orientation.w);
    // Move yaw to [-pi, pi]; as atan2 output is in [-pi, pi], yaw is initially in [-2*pi, 2*pi]
    if (yaw < -M_PI) yaw += 2*M_PI;
    if (yaw >  M_PI) yaw -= 2*M_PI;
    current_position_waypoint_ = {{_msg->pose.position.x, _msg->pose.position.y, _msg->pose.position.z}, yaw};
}*/

void UavStateMachine::positionCallback(const std_msgs::String::ConstPtr& _msg) {
	std::stringstream msg;
	msg << _msg->data;
	grvc::hal::Pose pose;
	msg >> pose;
    // TODO: Make an util for quaternion to euler conversion? Eigen?
    double yaw = 2*atan2(pose.orientation[2], pose.orientation[3]);
    // Move yaw to [-pi, pi]; as atan2 output is in [-pi, pi], yaw is initially in [-2*pi, 2*pi]
    if (yaw < -M_PI) yaw += 2*M_PI;
    if (yaw >  M_PI) yaw -= 2*M_PI;
    current_position_waypoint_ = {{pose.position[0], pose.position[1], pose.position[2]}, yaw};
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
            uav_state_machine::candidate target_candidate;
            target_candidate.color = target_.color;
            target_candidate.shape = target_.shape;
            target_candidate.global_position.x = target_.global_position.x;
            target_candidate.global_position.y = target_.global_position.y;
            if (bestCandidateMatch(candidateList, target_candidate, matched_candidate_)) {
                matched_candidate_.header.stamp = ros::Time::now();
                target_position_[0] = matched_candidate_.local_position.x;
                target_position_[1] = matched_candidate_.local_position.y;
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
    
    typedef std::pair<double, uav_state_machine::candidate> PairDistanceCandidate;
    std::vector<PairDistanceCandidate> pairsDistCands;

    Eigen::Vector3d specPos = {_specs.global_position.x, _specs.global_position.y, _specs.global_position.z};
    for (auto&candidate:_list.candidates) {
	bool isInField = frame_transform_.isInGameField( grvc::utils::constructPoint(   candidate.global_position.x,
                                                                                            candidate.global_position.y,
                                                                                            candidate.global_position.z) );
	if(isInField){
		Eigen::Vector3d candidatePos = {candidate.global_position.x, candidate.global_position.y, candidate.global_position.z};
		double dist = (specPos - candidatePos).norm();
		pairsDistCands.push_back(PairDistanceCandidate(dist, candidate));
	}
    }
    
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
