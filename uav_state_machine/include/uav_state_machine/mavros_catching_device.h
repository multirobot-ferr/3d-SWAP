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
#ifndef MBZIRC_MAVROS_CATCHING_DEVICE_H
#define MBZIRC_MAVROS_CATCHING_DEVICE_H

#include <uav_state_machine/catching_device.h>
#include <thread>
#include <chrono>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/RCOut.h>
#include <std_msgs/Bool.h>

class MavRosCatchingDevice: public CatchingDevice {
public:
    
    MavRosCatchingDevice(unsigned int _uav_id, ros::NodeHandle& _nh) {
        std::string magnet_topic = "/mavros_" + std::to_string(_uav_id) + "/actuator_control";
        magnet_publisher_ = _nh.advertise<mavros_msgs::ActuatorControl>(magnet_topic, 1);

        std::string switch_sub_topic = "/mavros_" + std::to_string(_uav_id) + "/rc/out";
        switch_subscriber_ = _nh.subscribe(switch_sub_topic, 1, &MavRosCatchingDevice::switchCallback, this);

        std::string magnetize_advertise = "/mbzirc_" + std::to_string(_uav_id) + "/catching_device/magnetize";
        magnetize_service_ = _nh.advertiseService(magnetize_advertise, &MavRosCatchingDevice::magnetizeServiceCallback, this);

        std::string switch_pub_topic = "/mbzirc_" + std::to_string(_uav_id) + "/catching_device/switch";
        switch_publisher_ = _nh.advertise<std_msgs::Bool>(switch_pub_topic, 1);
    }

    // Ask about magnet status
    MagnetState magnetState() { return magnet_state_; }

    // Ask about switch status
    bool switchIsPressed() { return switch_state_; }

    // Handle the magnet (may be up to ~4s blocking!)
    void setMagnetization(bool magnetize) {
        // Change magnetization only if needed
        MagnetState wanted_state = (magnetize? MagnetState::MAGNETIZED : MagnetState::DEMAGNETIZED);
        if (wanted_state != magnet_state_) {
            mavros_msgs::ActuatorControl control_signal;
            control_signal.group_mix = 3;
            control_signal.controls[6] = (magnetize ? 1.0 : -1.0);

            // Start magnetization/demagnetization
            std::chrono::time_point<std::chrono::steady_clock> t0 = std::chrono::steady_clock::now();
            magnet_publisher_.publish(control_signal);
            std::this_thread::sleep_for(std::chrono::milliseconds(3000));

            // Ensure that the magnet is resting
            control_signal.controls[6] = 0.0;
            magnet_publisher_.publish(control_signal);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            magnet_state_ = wanted_state;
        }
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
    bool switch_state_ = false;

    ros::Publisher magnet_publisher_;
    ros::Publisher switch_publisher_;
    ros::Subscriber switch_subscriber_;
    ros::ServiceServer magnetize_service_;

    void switchCallback(const mavros_msgs::RCOutConstPtr& _msg) {
        // Read from mavros...
        if (_msg->channels[8] < 1100)
            switch_state_ = false;
        else if (_msg->channels[8] > 1900)
            switch_state_ = true;
        // ...and republish!
        std_msgs::Bool pressed;
        pressed.data = switch_state_;
        switch_publisher_.publish(pressed);
    }
};

#endif  // MBZIRC_MAVROS_CATCHING_DEVICE_H
