#ifndef MBZIRC_CATCHING_DEVICE_H
#define MBZIRC_CATCHING_DEVICE_H

#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/RCOut.h>
#include <std_msgs/Bool.h>
#include <uav_state_machine/magnetize_service.h>

class CatchingDevice {
public:
    enum class MagnetState { UNKNOWN, MAGNETIZED, DEMAGNETIZED };
    
    CatchingDevice(unsigned int _uav_id, ros::NodeHandle& _nh) {
        std::string magnet_topic = "/mavros_" + std::to_string(_uav_id) + "/actuator_control";
        magnet_publisher_ = _nh.advertise<mavros_msgs::ActuatorControl>(magnet_topic, 1);

        std::string switch_sub_topic = "/mavros_" + std::to_string(_uav_id) + "/rc/out";
        switch_subscriber_ = _nh.subscribe(switch_sub_topic, 1, &CatchingDevice::switchCallback, this);

        std::string magnetize_advertise = "/mbzirc_" + std::to_string(_uav_id) + "/catching_device/magnetize";
        magnetize_service_ = _nh.advertiseService(magnetize_advertise, &CatchingDevice::magnetizeServiceCallback, this);

        std::string switch_pub_topic = "/mbzirc_" + std::to_string(_uav_id) + "/catching_device/switch";
        switch_publisher_ = _nh.advertise<std_msgs::Bool>(switch_pub_topic, 1);
    }

    // Ask about magnet status
    MagnetState magnetState() { return magnet_state_; }

    // Ask about switch status
    bool switchIsPressed() { return switch_state_; }

    // Handle the magnet (may be up to ~4s blocking!)
    bool magnetizeServiceCallback(uav_state_machine::magnetize_service::Request &_req,
         uav_state_machine::magnetize_service::Response &_res)
    {
        // Change magnetization only if needed
        MagnetState wanted_state = (_req.magnetize? MagnetState::MAGNETIZED : MagnetState::DEMAGNETIZED);
        if (wanted_state != magnet_state_) {
            mavros_msgs::ActuatorControl control_signal;
            control_signal.group_mix = 3;
            control_signal.controls[6] = (_req.magnetize ? 1.0 : -1.0);

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

#endif  // MBZIRC_CATCHING_DEVICE_H
