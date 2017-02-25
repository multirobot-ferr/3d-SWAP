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
#include <gazebo_msgs/LinkState.h>

class GazeboCatchingDevice: public CatchingDevice {
public:
    
    GazeboCatchingDevice(unsigned int _uav_id, ros::NodeHandle& _nh) {
        robot_link_name_ = "mbzirc_" + std::to_string(_uav_id) + "::base_link";

        std::string magnetize_advertise = "/mbzirc_" + std::to_string(_uav_id) + "/catching_device/magnetize";
        magnetize_service_ = _nh.advertiseService(magnetize_advertise, &GazeboCatchingDevice::magnetizeServiceCallback, this);

        std::string switch_pub_topic = "/mbzirc_" + std::to_string(_uav_id) + "/catching_device/switch";
        switch_publisher_ = _nh.advertise<std_msgs::Bool>(switch_pub_topic, 1);

        std::string link_states_sub_topic = "/gazebo/link_states";
        link_states_subscriber_ = _nh.subscribe(link_states_sub_topic, 1, &GazeboCatchingDevice::linkStatesCallback, this);

        std::string link_state_pub_topic = "/gazebo/set_link_state";
        link_state_publisher_ = _nh.advertise<gazebo_msgs::LinkState>(link_state_pub_topic, 1);

        pub_thread_ = std::thread([&](){
            while (ros::ok()) {
                // Release object if magnet is not magnetized
                if (magnet_state_ != MagnetState::MAGNETIZED) {
                    grabbing_ = false;
                }
                if (grabbing_) {
                    double z_grabbing = -0.5;  // TODO: as a param?
                    //std::cout << "Grabbing " << grabbed_link_name_ << std::endl;
                    gazebo_msgs::LinkState grabbed;
                    grabbed.link_name = grabbed_link_name_;
                    grabbed.pose.position.z = z_grabbing;
                    grabbed.reference_frame = robot_link_name_;
                    link_state_publisher_.publish(grabbed);
                } else if (magnet_state_ == MagnetState::MAGNETIZED) {
                    // Check distances between robot and grabbable objects
                    for (auto& name_pos : name_to_position_map_) {
                        Eigen::Vector3f diff = name_pos.second - robot_link_position_;
                        name_to_distance_map_[name_pos.first] = diff.norm();
                        std:: cout << "n2d[" << name_pos.first << "] = " << diff.norm() << std::endl;
                    }
                    // Find min distance in name_to_distance_map_...
                    auto min_distance_pair = std::min_element
                    (
                        std::begin(name_to_distance_map_), std::end(name_to_distance_map_),
                        [] (const std::pair<std::string, double> & p1, const std::pair<std::string, double> & p2) {
                            return p1.second < p2.second;
                        }
                    );
                    if (min_distance_pair != std::end(name_to_distance_map_)) {
                        // ... and check if its less than catching threshold TODO: as a param?
                        double catching_threshold = 1.0;
                        if (min_distance_pair->second < catching_threshold) {
                            grabbed_link_name_ = min_distance_pair->first;
                            grabbing_ = true;
                        }
                    }
                }
                // Publish switch state
                std_msgs::Bool switch_state;
                switch_state.data = grabbing_;
                switch_publisher_.publish(switch_state);
                // Sleep!
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });
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
    volatile bool grabbing_ = false;

    ros::Publisher switch_publisher_;
    ros::ServiceServer magnetize_service_;
    ros::Subscriber link_states_subscriber_;
    ros::Publisher link_state_publisher_;
    std::map<std::string, Eigen::Vector3f> name_to_position_map_;
    std::map<std::string, double> name_to_distance_map_;
    std::thread pub_thread_;

    Eigen::Vector3f robot_link_position_;
    std::string robot_link_name_;
    std::string grabbed_link_name_;

    void linkStatesCallback(const gazebo_msgs::LinkStatesConstPtr& _msg) {
        if (!grabbing_ && (magnet_state_ == MagnetState::MAGNETIZED)) {
            // All link states in world frame, find robot and grabbable objects
            for (size_t i = 0; i < _msg->name.size(); i++) {
                std::string link_name = _msg->name[i];
                if (link_name == robot_link_name_) {
                    robot_link_position_ << _msg->pose[i].position.x, \
                        _msg->pose[i].position.y, _msg->pose[i].position.z;
                }
                std::size_t found_grabbable = link_name.find("grab_here");
                if (found_grabbable != std::string::npos) {
                    Eigen::Vector3f link_position(_msg->pose[i].position.x, \
                        _msg->pose[i].position.y, _msg->pose[i].position.z);
                    name_to_position_map_[_msg->name[i]] = link_position;
                }
            }
        }
    }
};

#endif  // MBZIRC_GAZEBO_CATCHING_DEVICE_H
