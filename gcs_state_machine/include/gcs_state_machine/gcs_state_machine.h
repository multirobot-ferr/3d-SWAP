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
#ifndef MBZIRC_GCS_STATE_MACHINE_H
#define MBZIRC_GCS_STATE_MACHINE_H

#include <thread>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <uav_state_machine/UavState.h>

class GcsStateMachine {
	public:
		bool init(const std::vector<int> _uavsId);

	private:
		void onStateMachine();

		void onStateRepose();
		void onStateStart();
		void onStateSearching();
		void onStateCatching();
		void onStateEnd();
		void onStateError();

		// TODO: Use msg directly?
		enum class eGcsState { REPOSE, START, SEARCHING, CATCHING, END, ERROR };
		eGcsState 		gcs_state_ = eGcsState::REPOSE;
		std::string  	state_msg_;
		ros::Publisher	state_publisher_;
		std::thread 	state_publisher_thread_;

		std::thread state_machine_thread_;

		std::vector<uav_state_machine::UavState> uav_state_;
		std::vector<ros::Subscriber> uav_state_subscriber_;
		std::map<int, int> index_to_id_map_;
};

#endif  // MBZIRC_GCS_STATE_MACHINE_H
