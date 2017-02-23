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


bool GcsStateMachine::init(){
	state_publisher_thread_ = std::thread([&](){
		ros::NodeHandle nh;
		state_publisher_ = nh.advertise<gcs_state_machine::gcs_state>("/gcs/state", 1);
		while(ros::ok()){
			gcs_state_machine::gcs_state state;
			switch(gcs_state_){
				case eGcsState::STOP:
				state.state_str.data = "STOP";
				break;
			case eGcsState::SEARCHING:
				state.state_str.data = "SEARCHING";
				break;
			case eGcsState::CATCHING:
				state.state_str.data = "CATCHING";
				break;
			case eGcsState::ERROR:
				state.state_str.data = "ERROR";
				break;
			}
			state_publisher_.publish(state);
			std::this_thread::sleep_for(std::chrono::milliseconds(200));
		}
	});

	state_machine_thread_ = std::thread(&GcsStateMachine::onStateMachine, this);
	return false;
}

//-------------------------------------------------------------------------------------------------------------
void GcsStateMachine::onStateMachine(){
	while(ros::ok()){		
		switch(gcs_state_){
			case eGcsState::STOP:
				break;
			case eGcsState::SEARCHING:
				break;
			case eGcsState::CATCHING:
				break;
			case eGcsState::ERROR:
				break;
			default:
				assert(false);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}
}

