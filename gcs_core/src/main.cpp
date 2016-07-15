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

    void setPath(const WaypointList& _wp_list) {
        path_ = _wp_list;
    }

    void run() {
        assert(path_.size() > 0);

        // Wait for quad to be ready to fly
        while (!ready_) {
            this_thread::sleep_for(chrono::milliseconds(100));
        }

        grvc::uav::ActionState as;
        path_srv_->send(path_, as);
        cout << name_ << as <<"*************\n";
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

struct MovingObject {
public:
    MovingObject(const std::string& _object_name, const WaypointList& _wp_list, int _argc, char** _argv)
        : name_(_object_name)
        , path_(_wp_list)
    {
        path_srv_ = new grvc::hal::Server::PathService::Client(name_ + "/hal/path", utils::ArgumentParser(_argc, _argv));
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

        grvc::hal::TaskState ts;
        path_srv_->send(path_, ts);

    }

protected:
    hal::Server::PathService::Client* path_srv_ = nullptr;
    string name_;
    WaypointList path_;
    bool ready_ = false;
};

void object_thread(const string& _object_name, const WaypointList& _list, int _argc, char** _argv) {
    MovingObject object(_object_name, _list, _argc, _argv);
    object.run();
}


int main(int _argc, char** _argv) {
    utils::ArgumentParser parser(_argc, _argv);
    string quad = parser.getArgument("quad_name", string("quad1"));

    double flight_z = 10.0;

    WaypointList wplist_1 = {
        {{60.0, -25.0, flight_z}, 0.0},
        {{-50.0, -28.5, flight_z}, 0.0},
        {{-50.0, -22.5, flight_z}, 0.0},
        {{50.0, -22.5, flight_z}, 0.0}
    };

    Quadrotor quad1("/quad1", wplist_1, _argc, _argv);
    std::thread quad1_thread ([&](){
        quad1.run();
        quad1.setPath({
            {{7.0, -25, flight_z}, 0.0},
            {{7.0, -25, 1}, 0.0},
            {{7.0, -25, flight_z}, 0.0},
            {{-66.0, 25, flight_z}, 0.0},
            {{-66, 25.0, 5}, 0.0},
            {{-66, 25.0, 2}, 0.0},
            {{-66, 25.0, flight_z}, 0.0},
            {{-50.0, 20, flight_z}, 0.0}});
        quad1.run();
    });

    WaypointList wplist_3 = {
        {{60.0, 17.0, flight_z}, 0.0},
        {{-50.0, 17.0, flight_z}, 0.0},
        {{-50.0, 27.0, flight_z}, 0.0},
        {{50.0, 27.0, flight_z}, 0.0}

    };

    Quadrotor quad3("/quad3", wplist_3, _argc, _argv);
    std::thread quad3_thread ([&](){
        quad3.run();
        quad3.setPath({
            {{25.0, 22.0, flight_z}, 0.0},
            {{25.0, 22.0, 5}, 0.0},
            {{25.0, 22.0, 1}, 0.0},
            {{25.0, 22.0, flight_z}, 0.0},
            {{-50.0, 30.0, flight_z}, 0.0}});
        quad3.run();
    });

    this_thread::sleep_for(chrono::seconds(10));

    WaypointList wplist_2 = {
            {{60, -5.0, flight_z}, 0.0},
            {{-50, -5.0, flight_z}, 0.0},
            {{-50.0, 5.0, flight_z}, 0.0},
            {{50, 5.0, flight_z}, 0.0},
    };

    Quadrotor quad2("/quad2", wplist_2, _argc, _argv);
    std::thread quad2_thread ([&](){
        quad2.run();
        quad2.setPath({
            {{0, 0, flight_z}, 0.0},
            {{0, 0, 5}, 0.0},
            {{0, 0, 1}, 0.0}
          });
        quad2.run();
        this_thread::sleep_for(chrono::seconds(20));
        quad2.setPath({
            {{0, 0, 0.9}, 0.0},
            {{0, 0.0, flight_z}, 0.0},
            {{-50.0, 25, flight_z}, 0.0}});
        quad2.run();
    });

    quad1_thread.join();
    quad2_thread.join();
    quad3_thread.join();

    quad2.setPath({
                      {{-66, 25.0, flight_z}, 0.0},
                      {{-66, 25.0, 5}, 0.0},
                      {{-66, 25.0, 2}, 0.0},
                      {{-66, 25.0, flight_z}, 0.0},
                      {{-50.0, 25, flight_z}, 0.0}
                  });
    quad2.run();

    quad3.setPath({
                      {{-66, 25.0, flight_z}, 0.0},
                      {{-66, 25.0, 5}, 0.0},
                      {{-66, 25.0, 2}, 0.0},
                      {{-66, 25.0, flight_z}, 0.0},
                      {{-50.0, 30, flight_z}, 0.0}
                  });
    quad3.run();



    /*else if (quad == "object_black_cylinder_1") {
        WaypointList wplist_4 = {
            {{-40.0, -20.0, 0.06}, 0.0},
            {{40.0, -20.0, 0.06}, 0.0},
            {{40.0, 20.0, 0.06}, 0.0},
            {{-40.0, 20.0, 0.06}, 0.0}
        };
        object_thread("/object_black_cylinder_mov_1", wplist_4, _argc, _argv);
    }

    else if (quad == "object_black_cylinder_mov_2") {
        WaypointList wplist_5 = {
            {{17.0, 19.0, 0.06}, 0.0},
            {{17.0, -19.0, 0.06}, 0.0},
            {{17.0, 19.0, 0.06}, 0.0},
            {{17.0, -19.0, 0.06}, 0.0}
        };
        object_thread("/object_black_cylinder_mov_2", wplist_5, _argc, _argv);
    }

    else if (quad == "object_black_box_mov_1") {
        WaypointList wplist_6 = {
            {{-6.0, -13.0, 0.06}, 0.0},
            {{-18.0, -13.0, 0.06}, 0.0},
            {{-18.0, 13.0, 0.06}, 0.0},
            {{-6.0, 13.0, 0.06}, 0.0}
        };
        object_thread("/object_black_box_mov_1", wplist_6, _argc, _argv);
    }

    else if (quad == "object_black_box_mov_2") {
        WaypointList wplist_7 = {
            {{32.0, -12.0, 0.06}, 0.0},
            {{-32.0, -12.0, 0.06}, 0.0},
            {{-32.0, 12.0, 0.06}, 0.0},
            {{32.0, 12.0, 0.06}, 0.0}
        };
        object_thread("/object_black_box_mov_2", wplist_7, _argc, _argv);
    }

    else if (quad == "object_blue_cylinder_mov_1") {
        WaypointList wplist_8 = {
            {{40.0, 8.0, 0.06}, 0.0},
            {{40.0, -8.0, 0.06}, 0.0},
            {{10.0, -8.0, 0.06}, 0.0},
            {{10.0, 8.0, 0.06}, 0.0}
        };
        object_thread("/object_blue_cylinder_mov_1", wplist_8, _argc, _argv);
    }

    else if (quad == "object_blue_box_mov_1") {
        WaypointList wplist_9 = {
            {{-38.0, 17.0, 0.06}, 0.0},
            {{38.0, 17.0, 0.06}, 0.0},
            {{38.0, -17.0, 0.06}, 0.0},
            {{-38.0, -17.0, 0.06}, 0.0}
        };
        object_thread("/object_black_cylinder_mov_1", wplist_9, _argc, _argv);
    }

    else if (quad == "object_red_cylinder_mov_1") {
        WaypointList wplist_10 = {
            {{-28.0, 3.0, 0.06}, 0.0},
            {{3.0, 28.0, 0.06}, 0.0},
            {{28.0, -3.0, 0.06}, 0.0},
            {{-3.0, -28.0, 0.06}, 0.0}
        };
        object_thread("/object_red_cylinder_mov_1", wplist_10, _argc, _argv);
    }

    else if (quad == "object_red_cylinder_mov_2") {
        WaypointList wplist_11 = {
            {{13.0, -17.0, 0.06}, 0.0},
            {{-17.0, -13.0, 0.06}, 0.0},
            {{-13.0, 17.0, 0.06}, 0.0},
            {{17.0, 13.0, 0.06}, 0.0}
        };
        object_thread("/object_red_cylinder_mov_2", wplist_11, _argc, _argv);
    }

    else if (quad == "object_red_box_mov_1") {
        WaypointList wplist_12 = {
            {{18.0, -27.0, 0.06}, 0.0},
            {{-18.0, -27.0, 0.06}, 0.0},
            {{18.0, 27.0, 0.06}, 0.0},
            {{-18.0, 27.0, 0.06}, 0.0}
        };
        object_thread("/object_red_box_mov_1", wplist_12, _argc, _argv);
    }

    else if (quad == "object_red_box_mov_2") {
        WaypointList wplist_13 = {
            {{-22.0, -28.0, 0.06}, 0.0},
            {{-22.0, 28.0, 0.06}, 0.0},
            {{22.0, 28.0, 0.06}, 0.0},
            {{22.0, -28.0, 0.06}, 0.0}
        };
        object_thread("/object_red_box_mov_2", wplist_13, _argc, _argv);
    }*/

    return 0;
}
