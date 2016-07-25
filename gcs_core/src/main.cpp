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
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <grvc_com/ros/ros_singleton.h>
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

        ros_handle_ = com::RosSingleton::get()->handle();
        auto text_full_topic = name_ + "/" + "text";
        text_pub_ = ros_handle_->advertise<visualization_msgs::Marker>(text_full_topic.c_str(), 0);

        text_.header.frame_id = "/map";
        text_.ns = "text";
        text_.action = visualization_msgs::Marker::ADD;
        text_.pose.orientation.w = 1.0;
        text_.id = 0;
        text_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

        text_.text = name_;
        text_.scale.z = 0.5;
        if(name_ == "/quad1"){
            text_.color.r = 0.800f;
            text_.color.g = 0.242f;
            text_.color.b = 0.031f;
            text_.color.a = 1.0;
        }
        else if (name_ == "/quad2"){
            text_.color.r = 0.705f;
            text_.color.g = 0.135f;
            text_.color.b = 0.800f;
            text_.color.a = 1.0;
        }
        else if (name_ == "/quad3"){
            text_.color.r = 0.800f;
            text_.color.g = 0.715f;
            text_.color.b = 0.017f;
            text_.color.a =1.0;
        }

        new Subscriber<Vec3>(g_node_name, name_ + "/hal/position", _argc, _argv, [&](const Vec3& _v) {
          ready_ = true; // Once we receive telemetry, hal is ready
          text_.pose.position.x = _v[0];
          text_.pose.position.y = _v[1];
          text_.pose.position.z = _v[2] + 0.4;
          text_.header.stamp = ros::Time::now();
          text_pub_.publish(text_);  // TODO: decimate?
        });

    }

    void setPath(const WaypointList& _wp_list) {
        path_ = _wp_list;
    }

    void setText(const string& _in) {
      text_.text = _in;
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
    ros::NodeHandle* ros_handle_;
    ros::Publisher text_pub_;

    string name_;
    WaypointList path_;
    visualization_msgs::Marker text_;
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
        for(;;) {
            path_srv_->send(path_, ts);
        }

    }

protected:
    hal::Server::PathService::Client* path_srv_ = nullptr;
    string name_;
    WaypointList path_;
    bool ready_ = false;
};

void object_thread(const string& _object_name, const WaypointList& _list, int _argc, char** _argv) {
    MovingObject object(_object_name, _list, _argc, _argv);
    std::thread _thread1 ([&](){
        object.run();
    });
    _thread1.detach();
}

int main(int _argc, char** _argv) {
    utils::ArgumentParser parser(_argc, _argv);
    string quad = parser.getArgument("quad_name", string("quad1"));

    double flight_z = 10.0;

    WaypointList wplist_4 = {
        {{42.0, -24.0, 0.1}, 0.0},
        {{42.0, 24.0, 0.1}, 0.0},
        {{-42.0, 24.0, 0.1}, 0.0},
        {{-42.0, -24.0, 0.1}, 0.0}
    };
    MovingObject object1("/moving_object_2", wplist_4, _argc, _argv);
    std::thread _thread1 ([&](){
        object1.run();
    });

    WaypointList wplist_5 = {
        {{-15.0, 19.0, 0.1}, 0.0},
        {{-15.0, -19.0, 0.1}, 0.0}
    };
    MovingObject object2("/moving_object_3", wplist_5, _argc, _argv);
    std::thread _thread2 ([&](){
        object2.run();
    });

    WaypointList wplist_6 = {
        {{-6.0, -13.0, 0.1}, 0.0},
        {{-18.0, -13.0, 0.1}, 0.0},
        {{-18.0, 13.0, 0.1}, 0.0},
        {{-6.0, 13.0, 0.1}, 0.0}
    };
    MovingObject object3("/moving_object_4", wplist_6, _argc, _argv);
    std::thread _thread3 ([&](){
        object3.run();
    });

    WaypointList wplist_7 = {
        {{32.0, -12.0, 0.1}, 0.0},
        {{-32.0, -12.0, 0.1}, 0.0},
        {{-32.0, 12.0, 0.1}, 0.0},
        {{32.0, 12.0, 0.1}, 0.0}
    };
    MovingObject object4("/moving_object_5", wplist_7, _argc, _argv);
    std::thread _thread4 ([&](){
        object4.run();
    });

    WaypointList wplist_9 = {
        {{-38.0, 19.0, 0.1}, 0.0},
        {{38.0, 19.0, 0.1}, 0.0},
        {{38.0, -19.0, 0.1}, 0.0},
        {{-38.0, -19.0, 0.1}, 0.0}

    };
    MovingObject object6("/moving_object_6", wplist_9, _argc, _argv);
    std::thread _thread6 ([&](){
        object6.run();
    });

    WaypointList wplist_10 = {
        {{-12.0, 11.0, 0.1}, 0.0},
        {{-12.0, -11.0, 0.1}, 0.0},
        {{12.0, -11.0, 0.1}, 0.0},
        {{12.0, 11.0, 0.1}, 0.0}
    };
    MovingObject object7("/moving_object_7", wplist_10, _argc, _argv);
    std::thread _thread7 ([&](){
        object7.run();
    });

    WaypointList wplist_11 = {
        {{18.0, -27.0, 0.1}, 0.0},
        {{18.0, 27.0, 0.1}, 0.0}
    };
    MovingObject object8("/moving_object_8", wplist_11, _argc, _argv);
    std::thread _thread8 ([&](){
        object8.run();
    });

    WaypointList wplist_12 = {
        {{26.0, 0.0, 0.1}, 0.0},
        {{0.0, 26.0, 0.1}, 0.0},
        {{-26.0, 0.0, 0.1}, 0.0},
        {{0.0, -26.0, 0.1}, 0.0}
    };
    MovingObject object9("/moving_object_9", wplist_12, _argc, _argv);
    std::thread _thread9 ([&](){
        object9.run();
    });

    WaypointList wplist_13 = {
        {{0.0, 16.0, 0.1}, 0.0},
        {{16.0, 0.0, 0.1}, 0.0},
        {{0.0, -16.0, 0.1}, 0.0},
        {{-16.0, 0.0, 0.1}, 0.0}
    };
        MovingObject object0("/moving_object_10", wplist_13, _argc, _argv);
        std::thread _thread0 ([&](){
        object0.run();
    });

        //UAVs
    WaypointList wplist_1 = {
        {{65.0, -27.5, flight_z}, 0.0}
    };

    Quadrotor quad1("/quad1", wplist_1, _argc, _argv);
    std::thread quad1_thread ([&](){
        // Take off
        quad1.setText("taking-off");
        quad1.run();
        // Covering
        quad1.setPath({
          {{60.0, -27.5, flight_z}, 0.0},
          {{-50.0, -27.5, flight_z}, 0.0},
          {{-50.0, -22.5, flight_z}, 0.0},
          {{50.0, -22.5, flight_z}, 0.0}
        });
        quad1.setText("covering");
        quad1.run();
        // Catch
        quad1.setPath({
            {{7.0, -25, flight_z}, 0.0},
            {{7.0, -25, 0.9}, 0.0}});
        quad1.setText("catch!");
        quad1.run();
        //Success
        quad1.setPath({
            {{7.0, -25, flight_z}, 0.0}});
        quad1.setText("success!");
        quad1.run();
        // Release
        quad1.setPath({
            {{-50.0, 20, flight_z}, 0.0},
            {{-66.0, 25, flight_z}, 0.0},
            {{-66, 25.0, 5}, 0.0},
            {{-66, 25.0, 2}, 0.0},
            {{-66, 25.0, flight_z}, 0.0},
            {{-50.0, 20, flight_z}, 0.0}});
        quad1.setText("release");
        quad1.run();
    });

    WaypointList wplist_3 = {
        {{65.0, -22.5, flight_z}, 0.0}
    };

    Quadrotor quad3("/quad3", wplist_3, _argc, _argv);
    std::thread quad3_thread ([&](){
        // Take off
        quad3.setText("taking-off");
        quad3.run();
        // Covering
        quad3.setPath({
          {{60.0, 17.0, flight_z}, 0.0},
          {{-50.0, 17.0, flight_z}, 0.0},
          {{-50.0, 27.0, flight_z}, 0.0},
          {{50.0, 27.0, flight_z}, 0.0}});
        quad3.setText("covering");
        quad3.run();
        // Catch!
        quad3.setPath({
            {{25.0, 22.0, flight_z}, 0.0},
            {{25.0, 22.0, 5}, 0.0},
            {{25.0, 22.0, 0.9}, 0.0}});
        quad3.setText("catch!");
        quad3.run();
        //Success
        quad3.setPath({
            {{25.0, 22.0, flight_z}, 0.0}});
        quad3.setText("success!");
        quad3.run();
        //Release
        quad3.setPath({
            {{-50.0, 30.0, flight_z}, 0.0}});
        quad3.setText("release");
        quad3.run();
        quad3.setText("waiting");
    });

    this_thread::sleep_for(chrono::seconds(35));

    WaypointList wplist_2 = {
            {{65, -25.0, flight_z}, 0.0}
    };

    Quadrotor quad2("/quad2", wplist_2, _argc, _argv);
    std::thread quad2_thread ([&](){
        //Taking Off
        quad2.setText("taking-off");
        quad2.run();
        //Covering
        quad2.setPath({
            {{60, -5.0, flight_z}, 0.0},
            {{-50, -5.0, flight_z}, 0.0},
            {{-50.0, 5.0, flight_z}, 0.0},
            {{50, 5.0, flight_z}, 0.0}
          });
        quad2.setText("covering");
        quad2.run();
        //Catch
        quad2.setPath({
            {{0, 0, flight_z}, 0.0},
            {{0, 0, 5}, 0.0},
            {{0, 0, 1}, 0.0}
          });
        quad2.setText("catch!");
        quad2.run();
        //Success
        quad2.setPath({
            {{0, 0, 0.9}, 0.0},
            {{0, 0.0, flight_z}, 0.0}});
        this_thread::sleep_for(chrono::seconds(30));
        quad2.setText("success!");
        quad2.run();
        //Release
        quad2.setPath({
            {{-50.0, 25, flight_z}, 0.0}});
        quad2.setText("release");
        quad2.run();
        quad2.setText("waiting");
    });

    quad1_thread.join();
    quad2_thread.join();
    quad3_thread.join();

    std::thread quad1_1_thread ([&] (){
        quad1.setText("land");
        quad1.setPath({
            {{65.0, -26.5, flight_z}, 0.0},
            {{65.0, -26.5, 5}, 0.0},
            {{65.0, -26.5, 1}, 0.0},
            {{65.0, -26.5, 0.3}, 0.0}});
        quad1.run();
        quad1.setText("finished!");
    });

    quad3.setPath({
                      {{-66, 25.0, flight_z}, 0.0},
                      {{-66, 25.0, 5}, 0.0},
                      {{-66, 25.0, 2}, 0.0},
                      {{-66, 25.0, flight_z}, 0.0},
                      {{-50.0, 30, flight_z}, 0.0},
                      {{-50.0, 30, flight_z}, 0.0}
                  });
    quad3.setText("release");
    quad3.run();

    std::thread quad3_1_thread ([&](){
        quad3.setText("land");
        quad3.setPath({
            {{65, -23.5, flight_z}, 0.0},
            {{65, -23.5, 5}, 0.0},
            {{65, -23.5, 1}, 0.0},
            {{65, -23.5, 0.3}, 0.0}
          });
        quad3.run();
        quad3.setText("finished!");

    });

    quad2.setPath({
                      {{-66, 25.0, flight_z}, 0.0},
                      {{-66, 25.0, 5}, 0.0},
                      {{-66, 25.0, 2}, 0.0},
                      {{-66, 25.0, flight_z}, 0.0},
                      {{-50.0, 25, flight_z}, 0.0}
                  });
    quad2.setText("release");
    quad2.run();

    quad2.setPath({
                      {{65, -25, flight_z}, 0.0},
                      {{65, -25, 5}, 0.0},
                      {{65, -25.0, 1}, 0.0},
                      {{65, -25.0, 0.3}, 0.0}
                  });
    quad2.setText("land");
    quad2.run();
    quad2.setText("finished!");

    return 0;
}
