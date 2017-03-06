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

#ifndef GCS_STATE_MACHINE_H_
#define GCS_STATE_MACHINE_H_

#include <thread>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <uav_state_machine/uav_state.h>

class GcsStateMachine{
	public: // Public interface
		/// Initialize GCS state machine.
		bool init();

	private: // Private methods
		void onStateMachine();

		void onStateRepose();
		void onStateStart();
		void onStateSearching();
		void onStateCatching();
		void onStateEnd();
		void onStateError();
	private: // Members
		// Members related to CGS state.
		enum class eGcsState {REPOSE, START, SEARCHING, CATCHING, END, ERROR};
		eGcsState 		gcs_state_ = eGcsState::REPOSE;
		std::string  	state_msg_;
		ros::Publisher	state_publisher_;
		std::thread 	state_publisher_thread_;

		std::thread state_machine_thread_;

		std::array<uav_state_machine::uav_state, 3> uav_state_;
		std::array<ros::Subscriber, 3> uav_state_subscriber_;

		static inline geometry_msgs::Point createPoint(double _x, double _y, double _z) {
			geometry_msgs::Point p;
			p.x = _x;
			p.y = _y;
			p.z = _z;
			return p;
		}

};	// class GcsStateMachine


#endif
