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

class ObjectHaunter {
public:
    ObjectHaunter(int _argc, char** _argv) {
        string node_name = "take_object";
        grvc::utils::ArgumentParser args(_argc, _argv);
        piecePos_ = args.getArgument("start_pos", piecePos_);
        targetPos.model_name = "object_black_cylinder_1";
        targetPos.pose.position.x = piecePos_[0];
        targetPos.pose.position.y  = piecePos_[1];
        targetPos.pose.position.z  = piecePos_[2];
        //cout << piecePos_ << endl;
        ros::init(_argc, _argv, "haunter");
        n = new ros::NodeHandle();
        modelStatePub_ = n->advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 100);
        robotPosSub_ = new grvc::com::Subscriber<Vec3>(node_name.c_str(), "/quad2/hal/position", _argc, _argv, [this](const Vec3& _pos) {
            catchCallback(_pos);
        });
    }

protected:
    void catchCallback(const Vec3& _pos) {

        if((_pos - piecePos_).norm() < cCatchThreshold_ ) {
            if(!caught_ && !release)
            {
                cout << "Caught! -----------------\n";
                caught_ = true;
            }
        }
        if(caught_) {
            piecePos_ = _pos + Vec3(0.0, 0.0, -0.2);


            targetPos.pose.position.x = piecePos_[0];
            targetPos.pose.position.y = piecePos_[1];
            targetPos.pose.position.z = piecePos_[2];


            distanceBox_ = _pos - Vec3(-66, 25, 2);
            if(abs(distanceBox_[0]) < releaseThreshold_[0] && abs(distanceBox_[1]) < releaseThreshold_[1] && abs(distanceBox_[2]) < releaseThreshold_[2] )
            {
                caught_ = false;
                release = true;
                cout << "Object release-----------\n";
                //targetPos.twist.linear.z = -20;

            }
            cout << "Distance to box:" << distanceBox_[0] << " " << distanceBox_[1] << " " << distanceBox_[2] << endl;
        }
        if(release && targetPos.pose.position.z > 0.2)
        {
            targetPos.pose.position.z -= 0.1;


        }
        modelStatePub_.publish(targetPos);
    }

    gazebo_msgs::ModelState targetPos;
    Vec3 piecePos_;
    Vec3 releaseThreshold_ = Vec3(0.1, 0.1, 0.1);
    Vec3 distanceBox_;
    const double cCatchThreshold_ = 0.7f;
    bool release = false;
    bool caught_ = false;
    ros::NodeHandle* n;
    ros::Publisher modelStatePub_;
    grvc::com::Subscriber<Vec3>* robotPosSub_;

};




int main(int _argc, char** _argv)
{

    ObjectHaunter haunter(_argc, _argv);

    for(;;) {
         this_thread::sleep_for(chrono::milliseconds(500));
    }

    return 0;
}
