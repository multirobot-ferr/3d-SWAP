#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <gazebo_msgs/ModelState.h>
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

Server::WaypointService::Client* waypoint_srv;
int i = 0;

class MovingObject {
public:
    MovingObject(const grvc::utils::ArgumentParser& _args, int _argc, char** _argv) {
        string node_name = "moving_object";
        piece_pos_ = _args.getArgument("start_pos", piece_pos_);
        center_pos_ = piece_pos_;
        cur_vel_ = Vec3(3.0, 0.0, 0.0);
        piece_name_ = _args.getArgument("piece_name", string(""));
        string quad_name = _args.getArgument("quad_name", string(""));

        ros::init(_argc, _argv, "mobile_obj");
        n = new ros::NodeHandle();
        modelStatePub_ = n->advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 100);
        robotPosSub_ = new grvc::com::Subscriber<Vec3>(node_name, string("/")+quad_name+"/hal/position", _argc, _argv, [this](const Vec3& _pos) {
            catchCallback(_pos);
        });
    }

protected:
    void catchCallback(const Vec3& _pos) {
        const double cCatchThreshold = 1.0f;

        if((_pos - piece_pos_).norm() < cCatchThreshold ) {
            if(!caught_ && !released_)
            {
                cout << "Moving Caught! -----------------\n";
                caught_ = true;
            }
        }
        if(caught_) {
            piece_pos_ = _pos + Vec3(0.0, 0.0, -0.2); // Offset from quad

            Vec3 distanceToBox_ = _pos - Vec3(-66, 25, 2);
            if(abs(distanceToBox_[0]) < releaseThreshold_[0]
                && abs(distanceToBox_[1]) < releaseThreshold_[1] 
                && abs(distanceToBox_[2]) < releaseThreshold_[2] )
            {
                caught_ = false;
                released_ = true;
                cout << "Object release-----------\n";
                //targetPos.twist.linear.z = -20;
            }
            //cout << "Distance to box:" << distanceBox_[0] << " " << distanceBox_[1] << " " << distanceBox_[2] << endl;
        }
        if(released_ && piece_pos_.z() > 0.2)
        {
            piece_pos_.z() -= 0.1;
        }
        if(!caught_ && !released_) {
            cur_vel_ -= 0.02 * (piece_pos_ - center_pos_);
            piece_pos_ += 0.01 * cur_vel_;
        }

        gazebo_msgs::ModelState piece_state;
        piece_state.pose.position.x = piece_pos_.x();
        piece_state.pose.position.y = piece_pos_.y();
        piece_state.pose.position.z = piece_pos_.z();
        piece_state.model_name = piece_name_;
        modelStatePub_.publish(piece_state);
    }

    string piece_name_;
    Vec3 piece_pos_, center_pos_, cur_vel_;
    bool released_ = false;
    bool caught_ = false;

    Vec3 releaseThreshold_ = Vec3(0.1, 0.1, 0.1);
    ros::NodeHandle* n;
    ros::Publisher modelStatePub_;
    grvc::com::Subscriber<Vec3>* robotPosSub_;

};

int main(int _argc, char** _argv)
{
    grvc::utils::ArgumentParser args(_argc, _argv);
    MovingObject movingPiece(args, _argc, _argv);

    for(;;) {
         this_thread::sleep_for(chrono::milliseconds(500));
    }

    return 0;
}
