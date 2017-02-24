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
#include <std_msgs/Bool.h>

class GazeboCatchingDevice: public CatchingDevice {
public:
    
    GazeboCatchingDevice(unsigned int _uav_id, ros::NodeHandle& _nh) {
        std::string magnetize_advertise = "/mbzirc_" + std::to_string(_uav_id) + "/catching_device/magnetize";
        magnetize_service_ = _nh.advertiseService(magnetize_advertise, &GazeboCatchingDevice::magnetizeServiceCallback, this);

        std::string switch_pub_topic = "/mbzirc_" + std::to_string(_uav_id) + "/catching_device/switch";
        switch_publisher_ = _nh.advertise<std_msgs::Bool>(switch_pub_topic, 1);
    }

    // Ask about magnet status
    MagnetState magnetState() { return magnet_state_; }

    // Ask about switch status
    bool switchIsPressed() { return switch_state_; }

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
    bool switch_state_ = false;

    ros::Publisher switch_publisher_;
    ros::ServiceServer magnetize_service_;
};


#endif  // MBZIRC_GAZEBO_CATCHING_DEVICE_H
