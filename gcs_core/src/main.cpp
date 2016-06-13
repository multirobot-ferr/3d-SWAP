//----------------------------------------------------------------------------------------------------------------------
// GRVC Quadrotor HAL sample
//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
// 
// Copyright (c) 2016 GRVC University of Seville
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//----------------------------------------------------------------------------------------------------------------------
#include <grvc_com/publisher.h>
#include <grvc_quadrotor_hal/server.h>
#include <grvc_quadrotor_hal/types.h>
#include <grvc_utils/argument_parser.h>
#include <cstdint>
#include <thread>
#include <vector>
#include <string>
#include <thread>
#include <cassert>

using namespace grvc::com;
using namespace grvc::hal;
using namespace std;

const string g_node_name = "gcs_core";

struct Quadrotor {
	Quadrotor(const std::string& _quad_name, const WaypointList& _wp_list, int _argc, char** _argv) 
		: name_(_quad_name)
		, state_(TaskState::finished)
		, path_(_wp_list)
	{
		take_off_pub_ = new Publisher(g_node_name, name_ + "/hal/take_off", _argc ,_argv);
		path_pub_ = new Publisher(g_node_name, name_ + "/hal/path", _argc, _argv);

		new Subscriber<TaskState>(g_node_name, name_ + "/hal/state", _argc, _argv, [this](TaskState _s) {
			state_ = _s;
		});
		
		new Subscriber<Vec3>(g_node_name, name_ + "/hal/position", _argc, _argv, [&](const Vec3& _v) {
			ready_ = true; // Once we receive telemetry, hal is ready
			cur_z_ = _v.z();
		});
	}

	void run() {
		assert(path_.size() > 0);

		// Wait for quad to be ready to fly
		while (!ready_) {
			this_thread::sleep_for(chrono::milliseconds(100));
		}

		double fly_z = path_[0].pos.z();
		take_off_pub_->publish(fly_z, true);
		// This is a hack: ROS misses the first few state messages, so we have to check manually.
		while (cur_z_ < fly_z*0.8) {
			this_thread::sleep_for(chrono::milliseconds(100));
		}

		for (;;) {
			path_pub_->publish(path_);
			waitToFinish();
		}
	}

private:
	// Complete current task
	void waitToFinish() {
		while (state_ != TaskState::running) {
			this_thread::sleep_for(chrono::milliseconds(100));
		}
		while (state_ != TaskState::finished) {
			this_thread::sleep_for(chrono::milliseconds(100));
		}
	}

private:
	Publisher* take_off_pub_;
	Publisher* path_pub_;

	string name_;
	TaskState state_;
	WaypointList path_;

	double cur_z_;
	bool ready_ = false;
};

void quad_thread(const string& _quad_name, const WaypointList& _list, int _argc, char** _argv) {
	Quadrotor quad(_quad_name, _list, _argc, _argv);
	quad.run();
}

int main(int _argc, char** _argv) {
	grvc::utils::ArgumentParser parser(_argc, _argv);
	string quad = parser.getArgument("quad_name", string("quad1"));

	if(quad == "quad1") {
		WaypointList wplist_1 = {
			{{60.0, -25.0, 20}, 0.0},
			{{30.0, -20.0, 20}, 0.0},
			{{0.0, -20.0, 20}, 0.0},
			{{-30.0, -20.0, 20}, 0.0},
			{{-50.0, -20.0, 20}, 0.0},
			{{-50.0, 0.0, 20}, 0.0},
			{{-50.0, 20.0, 20}, 0.0},
			{{-30.0, 20.0, 20}, 0.0},
			{{0.0, 20.0, 20}, 0.0},
			{{30.0, 20.0, 20}, 0.0},
			{{-30.0, 20.0, 20}, 0.0},
			{{50.0, 20.0, 20}, 0.0}
		};

		quad_thread("/quad1", wplist_1, _argc, _argv);
	}
	else if (quad == "quad2") {
		WaypointList wplist_2 = {
			{{60, -15.0, 16}, 0.0},
			{{30, -5.0, 16}, 0.0},
			{{0.0, 0.0, 16}, 0.0},
			{{-30, 10.0, 16}, 0.0},
			{{-50, 20.0, 16}, 0.0},
			{{0, 20.0, 16}, 0.0},
			{{10, 0.0, 16}, 0.0},
			{{30, -5.0, 16}, 0.0},
			{{0.0, 0.0, 16}, 0.0},
			{{-30, 10.0, 16}, 0.0},
			{{-50, 20.0, 16}, 0.0},
			{{0, 20.0, 16}, 0.0},
			{{10, 0.0, 16}, 0.0}
		};
		
		quad_thread("/quad2", wplist_2, _argc, _argv);
	}

	else {
		WaypointList wplist_3 = {
			{{60.0, -10.0, 18}, 0.0},
			{{55.0, -10.0, 18}, 0.0},
			{{45.0, -5.0, 18}, 0.0},
			{{35.0, -10.0, 18}, 0.0},
			{{25.0, -5, 18}, 0.0},
			{{20.0, 0.0, 18}, 0.0},
			{{20.0, 5.0, 18}, 0.0},
			{{10.0, -5.0, 18}, 0.0},
			{{0.0, 5.0, 18}, 0.0},
			{{-10.0, 10.0, 18}, 0.0},
			{{-15.0, 10.0, 18}, 0.0},
			{{-25.0, 0.0, 18}, 0.0},
			{{-10.0, -10.0, 18.0}, 0.0}
		};
	
		quad_thread("/quad3", wplist_3, _argc, _argv);
	}

	return 0;
}
