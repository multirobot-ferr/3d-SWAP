//------------------------------------------------------------------------------
// GRVC MBZIRC
//------------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2016 GRVC University of Seville
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
//----------
#ifndef MBZIRC_GAZEBO_CATCHING_DEVICE_H
#define MBZIRC_GAZEBO_CATCHING_DEVICE_H

#include <uav_state_machine/catching_device.h>
#include <map>
#include <Eigen/Core>
#include <std_msgs/Bool.h>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Point.h>

class GazeboCatchingDevice: public CatchingDevice {
public:
    
    GazeboCatchingDevice(unsigned int _uav_id, ros::NodeHandle& _nh) {
        robot_name_ = "mbzirc_" + std::to_string(_uav_id);

        std::string magnetize_advertise = "/" + robot_name_ + "/catching_device/magnetize";
        magnetize_service_ = _nh.advertiseService(magnetize_advertise, &GazeboCatchingDevice::magnetizeServiceCallback, this);

        std::string switch_pub_topic = "/" + robot_name_ + "/catching_device/switch";
        switch_publisher_ = _nh.advertise<std_msgs::Bool>(switch_pub_topic, 1);

        std::string link_states_sub_topic = "/gazebo/link_states";
        link_states_subscriber_ = _nh.subscribe(link_states_sub_topic, 1, &GazeboCatchingDevice::linkStatesCallback, this);
    }

    // Ask about magnet status
    MagnetState magnetState() { return magnet_state_; }

    // Ask about switch status (won't give false positives)
    bool switchIsPressed() { return grabbing_; }

    // Handle the magnet
    void setMagnetization(bool magnetize) {
        magnet_state_ = (magnetize? MagnetState::MAGNETIZED : MagnetState::DEMAGNETIZED);
    }

    bool magnetizeServiceCallback(uav_state_machine::magnetize_service::Request &_req,
         uav_state_machine::magnetize_service::Response &_res)
    {
        setMagnetization(_req.magnetize);
        _res.success = true;
        return true;
    }

protected:
    MagnetState magnet_state_ = MagnetState::UNKNOWN;
    bool grabbing_ = false;

    ros::Publisher switch_publisher_;
    ros::ServiceServer magnetize_service_;
    ros::Subscriber link_states_subscriber_;
    std::map<std::string, Eigen::Vector3f> name_to_position_map_;
    std::map<std::string, double> name_to_distance_map_;

    std::string robot_name_;

    void linkStatesCallback(const gazebo_msgs::LinkStatesConstPtr& _msg) {
        if (!grabbing_) { // && (magnet_state_ == MagnetState::MAGNETIZED)) {
            // All link states in world frame, find robot and grabbable objects
            std::string robot_link_name = robot_name_ + "::base_link";
            Eigen::Vector3f robot_link_position;
            for (size_t i = 0; i < _msg->name.size(); i++) {
                std::string link_name = _msg->name[i];
                if (link_name == robot_link_name) {
                    robot_link_position << _msg->pose[i].position.x, \
                        _msg->pose[i].position.y, _msg->pose[i].position.z;
                }
                std::size_t found_grabbable = link_name.find("grab_here");
                if (found_grabbable != std::string::npos) {
                    Eigen::Vector3f link_position(_msg->pose[i].position.x, \
                        _msg->pose[i].position.y, _msg->pose[i].position.z);
                    name_to_position_map_[_msg->name[i]] = link_position;
                }
            }
            // Now check distances between robot and grabbable objects
            for (auto& name_pos : name_to_position_map_) {
                Eigen::Vector3f diff = name_pos.second - robot_link_position;
                name_to_distance_map_[name_pos.first] = diff.norm();
                std::cout << "n2p[" << name_pos.first << "] = " << name_pos.second << std::endl;  // debug
            }
            for (auto& name_dist : name_to_distance_map_) {
                std::cout << "n2d[" << name_dist.first << "] = " << name_dist.second << std::endl;  // debug
            }
        }
    }
};

#endif  // MBZIRC_GAZEBO_CATCHING_DEVICE_H
