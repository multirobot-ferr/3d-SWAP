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
#include <SerialStream.h>
#include <std_msgs/Bool.h>
#include <grvc_utils/critical.h>

#define MAGNET_PWM_DEMAGNETIZED 1000
#define MAGNET_PWM_REST 1520
#define MAGNET_PWM_MAGNETIZED 2000

#define PACKET_HEADER 0xFF
#define RX_PACKET_LENGTH 12
#define TX_PACKET_LENGTH 12
#define HEADER_LENGTH 2
#define RX_BUFFER_LENGTH 128
// [PACKET_HEADER][PACKET_HEADER][DATA_0][DATA_1]...[DATA_N][CHECKSUM_0][CHECKSUM_1]
// [    1byte    ][    1byte    ][1 byte][1 byte]...[1 byte][  1 byte  ][  1 byte  ]
// |--------------------------- PACKET_LENGTH, in bytes ---------------------------|

size_t uint16ToBuf(uint16_t input, char* buffer, uint16_t* checksum) {
    *buffer = (char)input;
    *checksum += *buffer;
    buffer++;

    *buffer = (char)(input >> 8);
    *checksum += *buffer;
    return 2;
}

class DeviceToPcData {
public:
    uint16_t sequence;
    uint16_t echo[2];
    bool switch_state;
    
    void setFrom(char* packet) {
        size_t i = HEADER_LENGTH;  // Skip packet header
        // First comes sequence
        this->sequence = packet[i] | (packet[i+1] << 8);
        i += 2;
        // Then echoes
        this->echo[0] = packet[i]   | (packet[i+1] << 8);
        this->echo[1] = packet[i+2] | (packet[i+3] << 8);
        i += 4;
        // Finally switch state
        this->switch_state = (packet[i] & 0x01)? true: false;
        //i += 2;
    }
};

class PcToDeviceData {
public:
    uint16_t sequence = 0;
    uint16_t pwm[2];

    void setTo(char* packet) {
        size_t i;
        uint16_t checksum = 0;
        // Header
        for (i = 0; i < HEADER_LENGTH; i++) {
            packet[i] = PACKET_HEADER;
        }
        // Sequence
        i += uint16ToBuf(this->sequence, &packet[i], &checksum);
        // Servos pwm
        i += uint16ToBuf(this->pwm[0], &packet[i], &checksum);
        i += uint16ToBuf(this->pwm[1], &packet[i], &checksum);
        // Padding
        uint16_t padding = 0;
        i += uint16ToBuf(padding, &packet[i], &checksum);
        // Checksum
        packet[i++] = (char)checksum;
        packet[i++] = (char)(checksum >> 8);
    }
};

class SerialCatchingDevice: public CatchingDevice {
public:
    
    SerialCatchingDevice(unsigned int _uav_id, ros::NodeHandle& _nh) {
        std::string magnetize_advertise = "/mbzirc_" + std::to_string(_uav_id) + "/catching_device/magnetize";
        magnetize_service_ = _nh.advertiseService(magnetize_advertise, &SerialCatchingDevice::magnetizeServiceCallback, this);

        std::string switch_pub_topic = "/mbzirc_" + std::to_string(_uav_id) + "/catching_device/switch";
        switch_publisher_ = _nh.advertise<std_msgs::Bool>(switch_pub_topic, 1);

        serial_thread_ = std::thread([this, _uav_id](){
            std::string serial_port;
            std::string param_name = "/mbzirc_" + std::to_string(_uav_id) + "/catching_device/serial_port";
            ros::param::param<std::string>(param_name, serial_port, "/dev/ttyUSB0");
            this->serial_.Open(serial_port);
            this->serial_.SetBaudRate(LibSerial::SerialStreamBuf::BAUD_115200);
            this->serial_.SetParity(LibSerial::SerialStreamBuf::PARITY_NONE);
            this->serial_.SetCharSize(LibSerial::SerialStreamBuf::CHAR_SIZE_8);
            this->serial_.SetFlowControl(LibSerial::SerialStreamBuf::FLOW_CONTROL_NONE);
            this->serial_.SetNumOfStopBits(1);
            if(!this->serial_.IsOpen()) {
                throw std::runtime_error("Couldn't open serial device!");
            }

            PcToDeviceData tx;
            tx.pwm[0] = MAGNET_PWM_REST;
            tx.pwm[1] = MAGNET_PWM_REST;
            char tx_packet[TX_PACKET_LENGTH];
            char rx = 0;
            char rx_packet[RX_PACKET_LENGTH];
            char rx_buffer[RX_BUFFER_LENGTH];
            size_t index = 0;
            uint16_t checksum = 0;
            enum State {INIT, SYNC, LOST, BODY, CHECK};
            
            State state = INIT;
          //while (this->serial_.IsOpen()) {
            while (ros::ok()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                // TX
                if (this->pwm_magnet_critical_.hasNewData()) {
                    tx.pwm[0] = pwm_magnet_critical_.get();
                }
                if (this->pwm_servo_critical_.hasNewData()) {
                    tx.pwm[1] = pwm_servo_critical_.get();
                }
                if (tx.sequence >= 0x0FFF) {
                    tx.sequence = 0;  // Force reset to keep it in 12 bits
                } else {
                    tx.sequence++;
                }
                tx.setTo(tx_packet);
                this->serial_.write(tx_packet, TX_PACKET_LENGTH);  // Always write!
                //std::cout << "sequence:" << tx.sequence << \
                //" pwm[0]:"  << tx.pwm[0] << \
                //" pwm[1]:"  << tx.pwm[1] << std::endl;
                // RX
                //size_t buffered = this->serial_.readsome(rx_buffer, RX_BUFFER_LENGTH);
                this->serial_.read(rx_buffer, RX_BUFFER_LENGTH);
                //for (size_t i = 0; i < buffered; i++) {
                for (size_t i = 0; i < RX_BUFFER_LENGTH; i++) {
                    rx = rx_buffer[i];
                    std::cout << std::oct << rx << std::endl;
                    switch (state) {

                        case INIT:
                            // index = 0;  // ...change index here is useless
                            checksum = 0;  // Don't break here, we're LOST indeed
                        case LOST:
                            if (rx == PACKET_HEADER) {
                                index = 1;  // ...buffering PACKET_HEADER is useless
                                state = SYNC;
                            } else {
                                state = LOST;
                            }
                            break;

                        case SYNC:
                            if (rx == PACKET_HEADER) {
                                index++;  // ...buffering PACKET_HEADER is useless
                            } else {
                                state = LOST;
                            }
                            // Whole header is HEADER_LENGTH bytes long
                            if (index >= HEADER_LENGTH) {
                                index = HEADER_LENGTH;
                                state = BODY;
                            }
                            break;

                        case BODY:
                            rx_packet[index++] = rx;  // Buffer byte...
                            checksum += rx;  // ...sum to checksum...
                            // ...and if we arrive to check...
                            if (index == RX_PACKET_LENGTH-2) {  // (checksum is 2 bytes long)
                                state = CHECK;			// ..change state!
                            }
                            break;

                        case CHECK:
                            rx_packet[index++] = rx;  // Buffer byte...
                            if (index == RX_PACKET_LENGTH) {  // ...and if we arrive to length, assure checksum...
                                if (checksum == (rx_packet[RX_PACKET_LENGTH-2] | (rx_packet[RX_PACKET_LENGTH-1]<<8))) {
                                    DeviceToPcData data;
                                    data.setFrom(rx_packet);
                                    this->rx_critical_.set(data);
                                } else {
                                    // TODO: throw checksum error?
                                }
                                state = INIT;  // ..and start again! (anyway)
                            }
                            break;

                        default:
                            state = INIT;  // But it's somekind of error
                            break;
                    }
                }
            }
        });

        pub_thread_ = std::thread([this](){
            while (ros::ok()) {
                if (this->rx_critical_.hasNewData()) {
                    DeviceToPcData rx = this->rx_critical_.get();
                    this->switch_state_ = rx.switch_state;
                    std::cout << "sequence:" << rx.sequence << \
                    " echo[0]:"  << rx.echo[0] << \
                    " echo[1]:"  << rx.echo[1] << \
                    " switch:"  << rx.switch_state << std::endl;
                    // Publish switch state
                    std_msgs::Bool switch_state;
                    switch_state.data = this->switch_state_;
                    this->switch_publisher_.publish(switch_state);
                } else {
                    // Sleep!
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
            }
        });

    }

    // TODO: Destructor closes serial_

    // Ask about magnet status
    MagnetState magnetState() { return magnet_state_; }

    // Ask about switch status
    bool switchIsPressed() { return switch_state_; }

    // Handle the magnet (may be up to ~4s blocking!)
    void setMagnetization(bool _magnetize) {
        // Change magnetization only if needed
        MagnetState wanted_state = (_magnetize? MagnetState::MAGNETIZED : MagnetState::DEMAGNETIZED);
        if (wanted_state != magnet_state_) {
            uint16_t pwm = (_magnetize ? MAGNET_PWM_MAGNETIZED : MAGNET_PWM_DEMAGNETIZED);
            // Start magnetization/demagnetization
            pwm_magnet_critical_.set(pwm);
            std::this_thread::sleep_for(std::chrono::milliseconds(3000));
            // Ensure that the magnet is resting
            pwm_magnet_critical_.set(MAGNET_PWM_REST);
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

    LibSerial::SerialStream serial_;
    std::thread serial_thread_;
    grvc::utils::Critical<DeviceToPcData> rx_critical_;
    grvc::utils::Critical<uint16_t> pwm_magnet_critical_;
    grvc::utils::Critical<uint16_t> pwm_servo_critical_;

    std::thread pub_thread_;
    ros::Publisher switch_publisher_;
    ros::ServiceServer magnetize_service_;
};

#endif  // MBZIRC_SERIAL_CATCHING_DEVICE_H
