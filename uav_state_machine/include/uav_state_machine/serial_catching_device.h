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
#ifndef MBZIRC_SERIAL_CATCHING_DEVICE_H
#define MBZIRC_SERIAL_CATCHING_DEVICE_H

#include <uav_state_machine/catching_device.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <std_msgs/Bool.h>
#include <grvc_utils/critical.h>

typedef unsigned char byte;
class DeviceToPcData {
public:
    uint16_t sequence;
    uint16_t echo[2];
    bool switch_state;
    
    void setFrom(byte* _packet);
};

class PcToDeviceData {
public:
    uint16_t sequence = 0;
    uint16_t pwm[2];

    void setTo(byte* _packet);
};

class SerialCatchingDevice: public CatchingDevice {
public:
    
    SerialCatchingDevice(unsigned int _uav_id, ros::NodeHandle& _nh);
    // TODO: Destructor closes serial

    // Ask about magnet status
    inline MagnetState magnetState() { return magnet_state_; }

    // Ask about switch status
    inline bool switchIsPressed() { return switch_state_; }

    // Handle the magnet (may be up to ~4s blocking!)
    void setMagnetization(bool _magnetize);

    bool magnetizeServiceCallback(uav_state_machine::magnetize_service::Request &_req,
         uav_state_machine::magnetize_service::Response &_res);

protected:
    MagnetState magnet_state_ = MagnetState::UNKNOWN;
    bool switch_state_ = false;

    grvc::utils::Critical<DeviceToPcData> rx_critical_;
    grvc::utils::Critical<uint16_t> pwm_magnet_critical_;
    grvc::utils::Critical<uint16_t> pwm_servo_critical_;

    std::thread pub_thread_;
    ros::Publisher switch_publisher_;
    ros::ServiceServer magnetize_service_;

    int serial_fd_;
    char* error_buffer_;
    std::thread rx_thread_;
    std::thread tx_thread_;
    enum State {INIT, SYNC, LOST, BODY, CHECK} rx_state_;
    byte* rx_buffer_;
    size_t rx_index_;
    uint16_t rx_checksum_;

    void openSerial(const std::string& _port_name, int _baud_rate);
    void closeSerial();
    void onRead(byte *_data, size_t _length);
    int serialWrite(byte *_data, size_t _length);
};

#endif  // MBZIRC_SERIAL_CATCHING_DEVICE_H
