//------------------------------------------------------------------------------
// GRVC MBZIRC 
//------------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2017 GRVC University of Seville
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
#include <gcs_state_machine/gcs_state_machine.h>
#include <gcs_state_machine/GcsState.h>

#include <uav_state_machine/SetTarget.h>
#include <uav_state_machine/TakeOff.h>
#include <uav_state_machine/Land.h>
#include <uav_state_machine/TrackPath.h>
// #include <mbzirc_scheduler/AssignTarget.h>
// #include <grvc_utils/frame_transform.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>

#define Z_SEARCHING 10.0

bool GcsStateMachine::init(const std::vector<int> _uavsId){
	state_publisher_thread_ = std::thread([&](){
		ros::NodeHandle nh;
		state_publisher_ = nh.advertise<gcs_state_machine::GcsState>("mbzirc_gcs/state", 1);
		while(ros::ok()){
			gcs_state_machine::GcsState state;
			state.state_msg.data = state_msg_.c_str();
			switch(gcs_state_){
				case eGcsState::REPOSE: 	state.state_str.data = "REPOSE"; 	break;
				case eGcsState::START: 		state.state_str.data = "START"; 	break;
				case eGcsState::SEARCHING: 	state.state_str.data = "SEARCHING"; break;
				case eGcsState::CATCHING: 	state.state_str.data = "CATCHING"; 	break;
				case eGcsState::END: 		state.state_str.data = "END"; 		break;
				case eGcsState::ERROR: 		state.state_str.data = "ERROR";		break;
			}
			state_publisher_.publish(state);
			std::this_thread::sleep_for(std::chrono::milliseconds(200));
		}
	});

	state_machine_thread_ = std::thread(&GcsStateMachine::onStateMachine, this);

	for (size_t i = 0; i < _uavsId.size(); i++) {
		index_to_id_map_[i] = _uavsId[i];
	}
	uav_state_subscriber_.resize(_uavsId.size());
	uav_state_.resize(_uavsId.size());

	ros::NodeHandle nh;
	for (size_t i = 0; i <index_to_id_map_.size(); i++) {
		std::string state_url = "mbzirc_" + std::to_string(index_to_id_map_[i]) + "/uav_state_machine/state";
		uav_state_subscriber_[i] = nh.subscribe<uav_state_machine::UavState>(
			state_url, 10,
			[this, i](const uav_state_machine::UavState::ConstPtr& _msg) {
				uav_state_[i] = *_msg;
		});
	}

	return true;
}


void GcsStateMachine::onStateMachine(){
	while(ros::ok()){		
		switch(gcs_state_){
			case eGcsState::REPOSE:
				onStateRepose();
				break;
			case eGcsState::START:
				onStateStart();
				break;
			case eGcsState::SEARCHING:
				onStateSearching();
				break;
			case eGcsState::CATCHING:
				onStateCatching();
				break;
			case eGcsState::END:
				onStateEnd();
				break;
			case eGcsState::ERROR:
				onStateError();
				break;
			default:
				assert(false);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}
}


void GcsStateMachine::onStateRepose(){
	int input=-1;
	std::cin >> input;
	if(input == 1) gcs_state_ = eGcsState::START;
}


void GcsStateMachine::onStateStart(){
	std::map<int, std::vector<geometry_msgs::PoseStamped>> start_path;

	// Prepare waypoints depending on the number of UAVs
	const double cEightHeight = 60;
	const double cEightWidth = 36;
	const double cFieldWidth = 60;
	
	switch(index_to_id_map_.size()){
		case 0:
			assert(false);
			break;
		case 1:
		{
			geometry_msgs::PoseStamped waypoint;
			waypoint.header.frame_id = "game";
			// The uav covers the whole area going forth&back.
			waypoint.pose.position.x = -cEightHeight/2;
			waypoint.pose.position.y = cFieldWidth/2 + cEightWidth/4;
			waypoint.pose.position.z = Z_SEARCHING;
			start_path[index_to_id_map_[0]].push_back(waypoint);

			waypoint.pose.position.x = cEightHeight/2;
			waypoint.pose.position.y = cFieldWidth/2 + cEightWidth/4;
			waypoint.pose.position.z = Z_SEARCHING;
			start_path[index_to_id_map_[0]].push_back(waypoint);

			waypoint.pose.position.x = cEightHeight/2;
			waypoint.pose.position.y = cFieldWidth/2 - cEightWidth/4;
			waypoint.pose.position.z = Z_SEARCHING;
			start_path[index_to_id_map_[0]].push_back(waypoint);

			waypoint.pose.position.x = -cEightHeight/2;
			waypoint.pose.position.y = cFieldWidth/2 - cEightWidth/4;
			waypoint.pose.position.z = Z_SEARCHING;
			start_path[index_to_id_map_[0]].push_back(waypoint);

			break;
		}
		case 2:
		{
			// The area is divided in two parts, each uav only move forward.
			
			int idMin, idMax;
			if(index_to_id_map_[0] > index_to_id_map_[1]){
				idMax = index_to_id_map_[0];
				idMin = index_to_id_map_[1];
			}else{
				idMin = index_to_id_map_[0];
				idMax = index_to_id_map_[1];
			}
			
			geometry_msgs::PoseStamped waypoint;
			waypoint.header.frame_id = "game";

			waypoint.pose.position.x = -cEightHeight/2;
			waypoint.pose.position.y = cFieldWidth/2 + cEightWidth/4;
			waypoint.pose.position.z = Z_SEARCHING;
			start_path[idMin].push_back(waypoint);

			waypoint.pose.position.x = cEightHeight/2;
			waypoint.pose.position.y = cFieldWidth/2 + cEightWidth/4;
			waypoint.pose.position.z = Z_SEARCHING;
			start_path[idMin].push_back(waypoint);
			
			waypoint.pose.position.x = -cEightHeight/2;
			waypoint.pose.position.y = cFieldWidth/2 - cEightWidth/4;
			waypoint.pose.position.z = Z_SEARCHING;
			start_path[idMax].push_back(waypoint);
			
			waypoint.pose.position.x = cEightHeight/2;
			waypoint.pose.position.y = cFieldWidth/2 - cEightWidth/4;
			waypoint.pose.position.z = Z_SEARCHING;
			start_path[idMax].push_back(waypoint);
			
			break;
		}
		case 3:
		{
			// The area is divided in three parts, each uav only move forth.
			
			geometry_msgs::PoseStamped waypoint;
			waypoint.header.frame_id = "game";

			waypoint.pose.position.x = -cEightHeight/2;
			waypoint.pose.position.y = cFieldWidth/2 + 2*cEightWidth/6;
			waypoint.pose.position.z = Z_SEARCHING;
			start_path[1].push_back(waypoint);
			
			waypoint.pose.position.x = cEightHeight/2;
			waypoint.pose.position.y = cFieldWidth/2 + 2*cEightWidth/6;
			waypoint.pose.position.z = Z_SEARCHING;
			start_path[1].push_back(waypoint);
			
			waypoint.pose.position.x = -cEightHeight/2;
			waypoint.pose.position.y = cFieldWidth/2;
			waypoint.pose.position.z = Z_SEARCHING;
			start_path[2].push_back(waypoint);

			waypoint.pose.position.x = cEightHeight/2;
			waypoint.pose.position.y = cFieldWidth/2;
			waypoint.pose.position.z = Z_SEARCHING;
			start_path[2].push_back(waypoint);
			
			waypoint.pose.position.x = -cEightHeight/2;
			waypoint.pose.position.y = cFieldWidth/2 - 2*cEightWidth/6;
			waypoint.pose.position.z = Z_SEARCHING;
			start_path[3].push_back(waypoint);

			waypoint.pose.position.x = cEightHeight/2;
			waypoint.pose.position.y = cFieldWidth/2 - 2*cEightWidth/6;
			waypoint.pose.position.z = Z_SEARCHING;
			start_path[3].push_back(waypoint);

			break;
		}
	}


	ros::NodeHandle nh;
	for (size_t i = 0; i < index_to_id_map_.size(); i++) {
		int id = index_to_id_map_[i];
		std::string takeoff_url = "/mbzirc_" + std::to_string(id) + "/uav_state_machine/takeoff";
		ros::ServiceClient takeoff_client = nh.serviceClient<uav_state_machine::TakeOff>(takeoff_url);
		uav_state_machine::TakeOff takeoff_call;
		takeoff_call.request.altitude = Z_SEARCHING;
		if (!takeoff_client.call(takeoff_call)) {
			state_msg_ = "Error taking off UAV_" + std::to_string(id);
		}
		/*while (uav_state_[i].state != uav_state_machine::UavState::HOVER) {
			// Wait until takeoff finishes
			std::this_thread::sleep_for(std::chrono::milliseconds(200));
		}*/
		std::string waypoint_url = "/mbzirc_" + std::to_string(id) + "/uav_state_machine/waypoint";
		ros::ServiceClient waypoint_client = nh.serviceClient<uav_state_machine::TrackPath>(waypoint_url);
		uav_state_machine::TrackPath waypoint_call;
		waypoint_call.request.action = uav_state_machine::TrackPathRequest::START;
		for (auto wp: start_path[id]) {
			waypoint_call.request.waypoint_track.push_back(wp);
		}
		if (!waypoint_client.call(waypoint_call)) {
			state_msg_ = "Error sending waypoints to UAV_" + std::to_string(id);
		}
		// Wait for a fixed time between UAVs
		std::this_thread::sleep_for(std::chrono::seconds(10));
	}
	// Arrived here, change state
	gcs_state_ = eGcsState::SEARCHING;
}


void GcsStateMachine::onStateSearching(){
	for (size_t i = 0; i < index_to_id_map_.size(); i++) {
		if (uav_state_[i].state != uav_state_machine::UavState::HOVER) {
			// Wait only until first UAV finishes starting path
			gcs_state_ = eGcsState::CATCHING;
			return;
		} else {
			std::this_thread::sleep_for(std::chrono::milliseconds(200));
		}
	}
	// TODO: Ask scheduler and repeat searching if no objects were found
}


void GcsStateMachine::onStateCatching(){
	ros::NodeHandle nh;
	// @Capi
	// ros::ServiceClient assign_target_client = nh.serviceClient<mbzirc_scheduler::AssignTarget>("/scheduler/assign_target");
	std::vector<ros::ServiceClient> catch_target_client;
	catch_target_client.resize(index_to_id_map_.size());
	for (size_t i = 0; i < index_to_id_map_.size(); i++) {
		std::string catch_target_url = "/mbzirc_" + std::to_string(index_to_id_map_[i]) + "/uav_state_machine/enabled";
		catch_target_client[i] = nh.serviceClient<uav_state_machine::SetTarget>(catch_target_url);
	}
	bool finished = false;
	while (!finished) {
		for (size_t i = 0; i < index_to_id_map_.size(); i++) {
			if (uav_state_[i].state == uav_state_machine::UavState::HOVER) {
				// @Capi
				// mbzirc_scheduler::AssignTarget assign_target_call;
				// assign_target_call.request.uav_id = index_to_id_map_[i];
				// if (!assign_target_client.call(assign_target_call)) {
				// 	//gcs_state_ = eGcsState::ERROR;  // No, retry!
				// 	state_msg_ = "Error calling assign target service in UAV_" + std::to_string(index_to_id_map_[i]);
				// } else if (assign_target_call.response.target_id >= 0) {
				// 	uav_state_machine::target_service catch_target_call;
				// 	catch_target_call.request.enabled = true;
				// 	catch_target_call.request.color = assign_target_call.response.color;
				// 	catch_target_call.request.shape = 0;  // DEPRECATED!
				// 	catch_target_call.request.target_id = assign_target_call.response.target_id;
				// 	catch_target_call.request.global_position = assign_target_call.response.global_position;
				// 	if (!catch_target_client[i].call(catch_target_call)) {
				// 		//gcs_state_ = eGcsState::ERROR;  // No, retry!
				// 		state_msg_ = "Error sending targets to UAV_" + std::to_string(index_to_id_map_[i]);
				// 	}
				// 	else
				// 		std::cout << "Target " << assign_target_call.response.target_id << " assigned to UAV " << (i+1) << std::endl;
				// } else {
				// 	// TODO: What? Finished?
				// 	std::cerr << "No valid target assigned!" << std::endl;
				// }
			}
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
		// TODO: Ask sheduler if whole mission is finished! (finished = true)
	}
	// We're done!
	gcs_state_ = eGcsState::END;
}


void GcsStateMachine::onStateEnd(){
	// TODO: Go to home, land and let's go party!
}


void GcsStateMachine::onStateError(){
	// TODO: Try to understand error situation and recover
}
