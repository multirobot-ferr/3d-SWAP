#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <stdio.h>
#include <iostream>
#include <std_msgs/String.h>
#include <grvc_quadrotor_hal/types.h>
#include <grvc_quadrotor_hal/server.h>
#include <string.h>
#include <thread>
#include <chrono>


using namespace std;
using namespace grvc::hal;

Vec3 g_piecePos;
const double cCatchThreshold = 0.5f;
bool caught = false;
Server::WaypointService::Client* waypoint_srv;
int i = 0;
void catchCallback(const Vec3& _pos) {

    if((_pos - g_piecePos).norm() < cCatchThreshold ) {
        if(!caught) cout << "Caught! -----------------\n";
        caught = true;
    }
    if(caught) {
        g_piecePos = _pos + Vec3(0.0, 0.0, -0.2);
    }
}

int main(int _argc, char** _argv)
{
    string node_name = "take_object";
    grvc::utils::ArgumentParser args(_argc, _argv);
    g_piecePos = args.getArgument("start_pos", g_piecePos);
    cout << g_piecePos << endl;

    waypoint_srv = new Server::WaypointService::Client(args.getArgument("gazebo_ns", string(""))+"/hal/go_to_wp", args);

    new grvc::com::Subscriber<Vec3>(node_name.c_str(), "/quad1/hal/position", _argc, _argv, catchCallback);

    for(;;) {
        TaskState ts;
        Waypoint wp = {g_piecePos, 0.0};
        waypoint_srv->send(wp, ts);
        this_thread::sleep_for(chrono::milliseconds(100));
    }

    return 0;
}
