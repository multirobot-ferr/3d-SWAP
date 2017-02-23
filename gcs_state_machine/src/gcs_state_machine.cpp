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
	ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<uav_state_machine::takeoff_service>("/mbzirc_1/uav_state_machine/takeoff");
	uav_state_machine::takeoff_service call;
	call.request.altitude = 4;
	auto res = client.call(call);
	if(!res) {
		gcs_state_ = eGcsState::ERROR;
		state_msg_ = "Error taking off UAV";
	}
	else{
		std::this_thread::sleep_for(std::chrono::seconds(5));
		gcs_state_ = eGcsState::SEARCHING;
	} 
}

//-------------------------------------------------------------------------------------------------------------
void GcsStateMachine::onStateSearching(){
	ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<uav_state_machine::waypoint_service>("/mbzirc_1/uav_state_machine/waypoint");
	uav_state_machine::waypoint_service req;
	req.request.action = 1;
	std::vector<std::array<double,3>> mWaypoints;
    //mWaypoints.push_back({0,0,0});
	//mWaypoints.push_back({1,0,-1});
	mWaypoints.push_back({0,1,1});
	for(auto wp: mWaypoints){
		geometry_msgs::Point p;
		p.x = wp[0];
		p.y = wp[1];
		p.z = wp[2];
		req.request.waypoint_track.push_back(p);
	}

	auto res = client.call(req);
	if(!res){
		gcs_state_ = eGcsState::ERROR;
		state_msg_ = "Error sending waypoints";
	}
	else{
		std::this_thread::sleep_for(std::chrono::seconds(5));
		gcs_state_ = eGcsState::CATCHING;
	} 
}

//-------------------------------------------------------------------------------------------------------------
void GcsStateMachine::onStateCatching(){
	ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<uav_state_machine::target_service>("/mbzirc_1/uav_state_machine/enabled");
    uav_state_machine::target_service call;
    call.request.enabled = true;
    call.request.color = 0;
    call.request.shape = 0;
    auto res = client.call(call);
	if(!res) {
		gcs_state_ = eGcsState::ERROR;
		state_msg_ = "Error Sending targets";
	}
	else gcs_state_ = eGcsState::END;
}

//-------------------------------------------------------------------------------------------------------------
void GcsStateMachine::onStateEnd(){

}

//-------------------------------------------------------------------------------------------------------------
void GcsStateMachine::onStateError(){

}


