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
#include <grvc_quadrotor_uav/server.h>
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
using namespace grvc;
using namespace std;

const string g_node_name = "gcs_core";

struct Quadrotor {
	Quadrotor(const std::string& _quad_name, const WaypointList& _wp_list, int _argc, char** _argv)
		: name_(_quad_name)
		, path_(_wp_list)
	{
		path_srv_ = new grvc::uav::Server::PathService::Client(name_ + "/uav/path", utils::ArgumentParser(_argc, _argv));
		new Subscriber<Vec3>(g_node_name, name_ + "/hal/position", _argc, _argv, [&](const Vec3& _v) {
			ready_ = true; // Once we receive telemetry, hal is ready
		});
	}

	void run() {
		assert(path_.size() > 0);

		// Wait for quad to be ready to fly
		while (!ready_) {
			this_thread::sleep_for(chrono::milliseconds(100));
		}

		grvc::uav::ActionState as;
		path_srv_->send(path_, as);

	}

private:
	grvc::uav::Server::PathService::Client* path_srv_;

	string name_;
	WaypointList path_;
	bool ready_ = false;
};

void quad_thread(const string& _quad_name, const WaypointList& _list, int _argc, char** _argv) {
	Quadrotor quad(_quad_name, _list, _argc, _argv);
	quad.run();
}

int main(int _argc, char** _argv) {
	utils::ArgumentParser parser(_argc, _argv);
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
