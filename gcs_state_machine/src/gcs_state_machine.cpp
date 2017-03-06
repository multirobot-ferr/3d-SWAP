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


#include <gcs_state_machine.h>
#include <gcs_state_machine/gcs_state.h>

#include <uav_state_machine/target_service.h>
#include <uav_state_machine/takeoff_service.h>
#include <uav_state_machine/land_service.h>
#include <uav_state_machine/waypoint_service.h>
#include <mbzirc_scheduler/AssignTarget.h>
#include <grvc_utils/frame_transform.h>

#define Z_SEARCHING 10.0

bool GcsStateMachine::init(){
	state_publisher_thread_ = std::thread([&](){
		ros::NodeHandle nh;
		state_publisher_ = nh.advertise<gcs_state_machine::gcs_state>("/gcs/state", 1);
		while(ros::ok()){
			gcs_state_machine::gcs_state state;
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

	ros::NodeHandle nh;
	for (int i = 0; i <3; i++) {
		std::string state_url = "/mbzirc_" + std::to_string(i+1) + "/uav_state_machine/state";
		uav_state_subscriber_[i] = nh.subscribe<uav_state_machine::uav_state>(
			state_url, 10,
			[this, i](const uav_state_machine::uav_state::ConstPtr& _msg) {
				uav_state_[i] = *_msg;  // WATCHOUT! 0-indexing with ids!
		});
	}

	return true;
}

//-------------------------------------------------------------------------------------------------------------
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

//-------------------------------------------------------------------------------------------------------------
void GcsStateMachine::onStateRepose(){
	int input=-1;
	std::cin >> input;
	if(input == 1) gcs_state_ = eGcsState::START;
}

//-------------------------------------------------------------------------------------------------------------
void GcsStateMachine::onStateStart(){
	std::array<std::vector<geometry_msgs::Point>, 3> start_path;
	// TODO: Solve game-map transformation problem... load from file?
	grvc::utils::frame_transform frameTransform;

	start_path[0].push_back(frameTransform.game2map(grvc::utils::constructPoint(-38.33, 46.67, Z_SEARCHING)));
	start_path[0].push_back(frameTransform.game2map(grvc::utils::constructPoint(+38.33, 46.67, Z_SEARCHING)));
	start_path[0].push_back(frameTransform.game2map(grvc::utils::constructPoint(+38.33, 53.33, Z_SEARCHING)));
	start_path[0].push_back(frameTransform.game2map(grvc::utils::constructPoint(-38.33, 53.33, Z_SEARCHING)));

	start_path[1].push_back(frameTransform.game2map(grvc::utils::constructPoint(-38.33, 33.33, Z_SEARCHING)));
	start_path[1].push_back(frameTransform.game2map(grvc::utils::constructPoint(+38.33, 33.33, Z_SEARCHING)));
	start_path[1].push_back(frameTransform.game2map(grvc::utils::constructPoint(+38.33, 26.67, Z_SEARCHING)));
	start_path[1].push_back(frameTransform.game2map(grvc::utils::constructPoint(-38.33, 26.67, Z_SEARCHING)));

	start_path[2].push_back(frameTransform.game2map(grvc::utils::constructPoint(-38.33, 13.33, Z_SEARCHING)));
	start_path[2].push_back(frameTransform.game2map(grvc::utils::constructPoint(+38.33, 13.33, Z_SEARCHING)));
	start_path[2].push_back(frameTransform.game2map(grvc::utils::constructPoint(+38.33,  6.67, Z_SEARCHING)));
	start_path[2].push_back(frameTransform.game2map(grvc::utils::constructPoint(-38.33,  6.67, Z_SEARCHING)));

	ros::NodeHandle nh;
	std::array<int, 3> sequence = {1, 3, 2};  // Launching order
	for (int j = 0; j < 3; j++) {
		int id = sequence[j];
		std::string takeoff_url = "/mbzirc_" + std::to_string(id) + "/uav_state_machine/takeoff";
		ros::ServiceClient takeoff_client = nh.serviceClient<uav_state_machine::takeoff_service>(takeoff_url);
		uav_state_machine::takeoff_service takeoff_call;
		takeoff_call.request.altitude = Z_SEARCHING;
		if (!takeoff_client.call(takeoff_call)) {
			gcs_state_ = eGcsState::ERROR;
			state_msg_ = "Error taking off UAV_" + std::to_string(id);
			return;
		}
		while (uav_state_[id-1].state != uav_state_machine::uav_state::HOVER) {
			// Wait until takeoff finishes
			std::this_thread::sleep_for(std::chrono::milliseconds(200));
		}
		std::string waypoint_url = "/mbzirc_" + std::to_string(id) + "/uav_state_machine/waypoint";
		ros::ServiceClient waypoint_client = nh.serviceClient<uav_state_machine::waypoint_service>(waypoint_url);
		uav_state_machine::waypoint_service waypoint_call;
		waypoint_call.request.action = uav_state_machine::waypoint_serviceRequest::START;
		for (auto wp: start_path[id-1]) {
			waypoint_call.request.waypoint_track.push_back(wp);
		}
		if (!waypoint_client.call(waypoint_call)) {
			gcs_state_ = eGcsState::ERROR;
			state_msg_ = "Error sending waypoints to UAV_" + std::to_string(id);
			return;
		}
	}
	// Arrived here, change state
	gcs_state_ = eGcsState::SEARCHING;
}

//-------------------------------------------------------------------------------------------------------------
void GcsStateMachine::onStateSearching(){
	for (int i = 0; i < 3; i++) {
		while (uav_state_[i].state != uav_state_machine::uav_state::HOVER) {
			// Wait all UAV to finish starting path
			std::this_thread::sleep_for(std::chrono::milliseconds(200));
		}
		// TODO: Go to catching level instead of wait until other finishes? (sequence_?)
	}

	// TODO: Ask scheduler and repeat searching if no objects were found
	gcs_state_ = eGcsState::CATCHING;
}

//-------------------------------------------------------------------------------------------------------------
void GcsStateMachine::onStateCatching(){
	ros::NodeHandle nh;
	ros::ServiceClient assign_target_client = nh.serviceClient<mbzirc_scheduler::AssignTarget>("/scheduler/assign_target");
	std::array<ros::ServiceClient, 3> catch_target_client;
	for (int i = 0; i < 3; i++) {
		std::string catch_target_url = "/mbzirc_" + std::to_string(i+1) + "/uav_state_machine/enabled";
		catch_target_client[i] = nh.serviceClient<uav_state_machine::target_service>(catch_target_url);
	}
	bool finished = false;
	while (!finished) {
		for (int i = 0; i < 3; i++) {
			if (uav_state_[i].state == uav_state_machine::uav_state::HOVER) {
				mbzirc_scheduler::AssignTarget assign_target_call;
				assign_target_call.request.uav_id = i+1;
				if (!assign_target_client.call(assign_target_call)) {
					//gcs_state_ = eGcsState::ERROR;  // No, retry!
					state_msg_ = "Error calling assign target service in UAV_" + std::to_string(i+1);
				} else {
					uav_state_machine::target_service catch_target_call;
					catch_target_call.request.enabled = true;
					catch_target_call.request.color = assign_target_call.response.color;
					catch_target_call.request.shape = 0;  // DEPRECATED!
					catch_target_call.request.target_id = assign_target_call.response.target_id;
					catch_target_call.request.global_position = assign_target_call.response.global_position;
					//catch_target_call.request.local_position = assign_target_call.response.local_position;  // TODO: wtf!
					if (!catch_target_client[i].call(catch_target_call)) {
						gcs_state_ = eGcsState::ERROR;
						state_msg_ = "Error sending targets to UAV_" + std::to_string(i+1);
					}
				}
			}
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
		// TODO: Ask sheduler if whole mission is finished! (finished = true)
	}
	// We're done!
	gcs_state_ = eGcsState::END;
}

//-------------------------------------------------------------------------------------------------------------
void GcsStateMachine::onStateEnd(){
	// TODO: Go to home, land and let's go party!
}

//-------------------------------------------------------------------------------------------------------------
void GcsStateMachine::onStateError(){
	// TODO: Try to understand error situation and recover
}


