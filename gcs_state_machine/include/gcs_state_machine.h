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

class GcsStateMachine{
	public: // Public interface
		/// Initialize GCS state machine.
		bool init();

	private: // Private methods
		void onStateMachine();

	private: // Members
		// Members related to CGS state.
		enum class eGcsState {STOP, SEARCHING, CATCHING, ERROR};
		eGcsState 	gcs_state_ = eGcsState::STOP;
		ros::Publisher	state_publisher_;
		std::thread 	state_publisher_thread_;

		std::thread state_machine_thread_;



};	// class GcsStateMachine


#endif
