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
#include <uav_state_machine/serial_catching_device.h>
#include <termios.h>
#include <poll.h>
#include <fcntl.h>
#include <fstream>
#include <string>

#define MAGNET_PWM_DEMAGNETIZED 1000
#define MAGNET_PWM_REST 1500
#define MAGNET_PWM_MAGNETIZED 2000
#define SERVO_PWM_NEUTRAL 1520

#define PACKET_HEADER 0xFF
#define RX_PACKET_LENGTH 12
#define TX_PACKET_LENGTH 12
#define HEADER_LENGTH 2
#define RX_BUFFER_LENGTH 128
#define ERROR_BUFFER_LENGTH 1024

size_t uint16ToBuf(uint16_t _input, byte* _buffer, uint16_t* _checksum) {
    *_buffer = (byte)_input;
    *_checksum += *_buffer;
    _buffer++;

    *_buffer = (byte)(_input >> 8);
    *_checksum += *_buffer;
    return 2;
}

// [PACKET_HEADER][PACKET_HEADER][DATA_0][DATA_1]...[DATA_N][CHECKSUM_0][CHECKSUM_1]
// [    1byte    ][    1byte    ][1 byte][1 byte]...[1 byte][  1 byte  ][  1 byte  ]
// |--------------------------- PACKET_LENGTH, in bytes ---------------------------|

void DeviceToPcData::setFrom(byte* _packet) {
    size_t i = HEADER_LENGTH;  // Skip packet header
    // First comes sequence
    this->sequence = _packet[i] | (_packet[i+1] << 8);
    i += 2;
    // Then echoes
    this->echo[0] = _packet[i]   | (_packet[i+1] << 8);
    this->echo[1] = _packet[i+2] | (_packet[i+3] << 8);
    i += 4;
    // Finally switch and magnet state
    this->switch_state = (_packet[i] & 0x01)? true: false;
    //this->magnet_state = CatchingDevice::MagnetState::UNKNOWN;
    uint8_t magnet_int = _packet[i+1];
    switch (magnet_int) {
        case 0:
            this->magnet_state = CatchingDevice::MagnetState::UNKNOWN;
            break;
        case 1:
            this->magnet_state = CatchingDevice::MagnetState::MAGNETIZED;
            break;
        case 2:
            this->magnet_state = CatchingDevice::MagnetState::DEMAGNETIZED;
            break;
        default:
            this->magnet_state = CatchingDevice::MagnetState::UNKNOWN;  // But it's an error
            break;
    }
    //i += 2;
}

void PcToDeviceData::setTo(byte* _packet) {
    size_t i;
    uint16_t checksum = 0;
    // Header
    for (i = 0; i < HEADER_LENGTH; i++) {
        _packet[i] = PACKET_HEADER;
    }
    // Sequence
    i += uint16ToBuf(this->sequence, &_packet[i], &checksum);
    // Servos pwm
    i += uint16ToBuf(this->pwm[0], &_packet[i], &checksum);
    i += uint16ToBuf(this->pwm[1], &_packet[i], &checksum);
    // Padding
    uint16_t padding = 0;
    i += uint16ToBuf(padding, &_packet[i], &checksum);
    // Checksum
    _packet[i++] = (byte)checksum;
    _packet[i++] = (byte)(checksum >> 8);
}

SerialCatchingDevice::SerialCatchingDevice(unsigned int _uav_id, ros::NodeHandle& _nh) {
    std::string magnetize_advertise = "/mbzirc_" + std::to_string(_uav_id) + "/catching_device/magnetize";
    magnetize_service_ = _nh.advertiseService(magnetize_advertise, &SerialCatchingDevice::magnetizeServiceCallback, this);

    std::string switch_pub_topic = "/mbzirc_" + std::to_string(_uav_id) + "/catching_device/switch";
    switch_publisher_ = _nh.advertise<std_msgs::Bool>(switch_pub_topic, 1);

    std::string serial_port;
    std::string param_name = "/mbzirc_" + std::to_string(_uav_id) + "/catching_device/serial_port";
    ros::param::param<std::string>(param_name, serial_port, "/dev/ttyUSB0");
    error_buffer_ = new char[ERROR_BUFFER_LENGTH];
    rx_buffer_ = new byte[RX_BUFFER_LENGTH];
    openSerial(serial_port, 115200);

    rx_state_ = INIT;
    rx_thread_ = std::thread([this](){
        struct pollfd ufd[1];
        ufd[0].fd = this->serial_fd_;
        ufd[0].events = POLLIN;
        while (ros::ok()) {
            if (poll(ufd, 1, 10) > 0) {
                if (!(ufd[0].revents & POLLERR)) {
                    int ret = ::read(this->serial_fd_, this->rx_buffer_, RX_BUFFER_LENGTH);
                    if (ret > 0) { this->onRead(this->rx_buffer_, ret); }
                }
            } 
        }
    });

    tx_thread_ = std::thread([this](){
        byte tx_packet[TX_PACKET_LENGTH];
        PcToDeviceData tx;
        tx.pwm[0] = MAGNET_PWM_REST;
        tx.pwm[1] = SERVO_PWM_NEUTRAL;
        while(ros::ok()){
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
            serialWrite(tx_packet, TX_PACKET_LENGTH);  // Always write!
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });

    pub_thread_ = std::thread([this](){
        while (ros::ok()) {
            if (this->rx_critical_.hasNewData()) {
                DeviceToPcData rx = this->rx_critical_.get();
                this->switch_state_ = rx.switch_state;
                this->magnet_state_ = rx.magnet_state;
                /*std::string magnet_state = "error";
                switch (rx.magnet_state) {
                    case MagnetState::UNKNOWN:
                        magnet_state = "unknown";
                        break;
                    case MagnetState::MAGNETIZED:
                        magnet_state = "magnetized";
                        break;
                    case MagnetState::DEMAGNETIZED:
                        magnet_state = "demagnetized";
                        break;
                    default:
                        break;
                }
                std::cout << "sequence:" << rx.sequence << \
                " echo[0]:"  << rx.echo[0] << \
                " echo[1]:"  << rx.echo[1] << \
                " switch:"  << rx.switch_state << \
                " magnet:" << magnet_state << std::endl;*/
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

void SerialCatchingDevice::setMagnetization(bool _magnetize) {
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
        //magnet_state_ = wanted_state;  // Now magnet state comes from serial device!
    }
}

bool SerialCatchingDevice::magnetizeServiceCallback(uav_state_machine::Magnetize::Request &_req,
        uav_state_machine::Magnetize::Response &_res)
{
    setMagnetization(_req.magnetize);
    _res.success = true;
    return true;
}

void SerialCatchingDevice::openSerial(const std::string& _port_name, int _baud_rate) {
    serial_fd_ = ::open(_port_name.c_str(), O_RDWR | O_NONBLOCK | O_NOCTTY);
    if (serial_fd_ == -1) {
        std::string extra_msg;
        switch (errno) {
            case EACCES:
            extra_msg = "You probably don't have premission to open the port for reading and writing.";
            break;

            case ENOENT:
            extra_msg = "The requested port does not exist. Is the device connected? \
            Was the port name misspelled?";
            break;
        }
        snprintf(error_buffer_, ERROR_BUFFER_LENGTH, \
        "[in function %s]::Failed to open port: %s. %s (errno = %d). %s", \
        __FUNCTION__, _port_name.c_str(), strerror(errno), errno, extra_msg.c_str());
        throw std::runtime_error(error_buffer_);
    }
    try {
        struct flock fl;
        fl.l_type = F_WRLCK;
        fl.l_whence = SEEK_SET;
        fl.l_start = 0;
        fl.l_len = 0;
        fl.l_pid = getpid();
        if (fcntl(serial_fd_, F_SETLK, &fl) != 0) {
            snprintf(error_buffer_, ERROR_BUFFER_LENGTH, \
            "[in function %s]::Device %s is already locked. \
            Try 'lsof | grep %s' to find other processes that currently have the port open.",\
                __FUNCTION__, _port_name.c_str(), _port_name.c_str());
            throw std::runtime_error(error_buffer_);
        }
        // Settings for USB?
        struct termios newtio;
        tcgetattr(serial_fd_, &newtio);
        memset (&newtio.c_cc, 0, sizeof (newtio.c_cc));
        newtio.c_cflag = CS8 | CLOCAL | CREAD;
        newtio.c_iflag = IGNPAR;
        newtio.c_oflag = 0;
        newtio.c_lflag = 0;
        cfsetspeed(&newtio, _baud_rate);
        // Activate new settings
        tcflush(serial_fd_, TCIFLUSH);
        if(tcsetattr(serial_fd_, TCSANOW, &newtio) < 0) {
            snprintf(error_buffer_, ERROR_BUFFER_LENGTH, \
            "[in function %s]::Unable to set serial port attributes. \
            The port you specified (%s) may not be a serial port.", __FUNCTION__, _port_name.c_str());
            throw std::runtime_error(error_buffer_);
            // TODO: tcsetattr returns true if at least one attribute was set.
            // Hence, we might not have set everything on success.
        }
        usleep (200000);
    } catch(std::runtime_error& e) {
        // These exceptions mean something failed on open and we should close
        if(serial_fd_ != -1) ::close(serial_fd_);
        serial_fd_ = -1;
        throw e;
    }
}

void SerialCatchingDevice::closeSerial() {
    int retval = 0;
    retval = ::close(serial_fd_);
    serial_fd_ = -1;
    if(retval != 0) {
        snprintf(error_buffer_, ERROR_BUFFER_LENGTH, \
        "[in function %s]::Failed to close port properly -- error = %d: %s", \
        __FUNCTION__, errno, strerror(errno));
        throw std::runtime_error(error_buffer_);
    }
}

void SerialCatchingDevice::onRead(byte *_data, size_t _length) {
    byte rx = 0;
    byte rx_packet[RX_PACKET_LENGTH];
    for (size_t i = 0; i < _length; i++) {
        rx = _data[i];
        switch (rx_state_) {

            case INIT:
                // rx_index_ = 0;  // ...change index here is useless
                rx_checksum_ = 0;  // Don't break here, we're LOST indeed
                //std::cout << "INIT" << std::endl;
            case LOST:
                if (rx == PACKET_HEADER) {
                    rx_index_ = 1;  // ...buffering PACKET_HEADER is useless
                    rx_state_ = SYNC;
                } else {
                    rx_state_ = LOST;
                }
                //std::cout << "LOST" << std::endl;
                break;

            case SYNC:
                if (rx == PACKET_HEADER) {
                    rx_index_++;  // ...buffering PACKET_HEADER is useless
                } else {
                    rx_state_ = LOST;
                }
                // Whole header is HEADER_LENGTH bytes long
                if (rx_index_ >= HEADER_LENGTH) {
                    rx_index_ = HEADER_LENGTH;
                    rx_state_ = BODY;
                }
                //std::cout << "SYNC" << std::endl;
                break;

            case BODY:
                rx_packet[rx_index_++] = rx;  // Buffer byte...
                rx_checksum_ += rx;  // ...sum to checksum...
                // ...and if we arrive to check...
                if (rx_index_ == RX_PACKET_LENGTH-2) {  // (checksum is 2 bytes long)
                    rx_state_ = CHECK;			// ..change state!
                }
                //std::cout << "BODY" << std::endl;
                break;

            case CHECK:
                rx_packet[rx_index_++] = rx;  // Buffer byte...
                if (rx_index_ == RX_PACKET_LENGTH) {  // ...and if we arrive to length, assure checksum...
                    if (rx_checksum_ == (rx_packet[RX_PACKET_LENGTH-2] | (rx_packet[RX_PACKET_LENGTH-1]<<8))) {
                        DeviceToPcData data;
                        data.setFrom(rx_packet);
                        this->rx_critical_.set(data);
                    } else {
                        // TODO: throw checksum error?
                    }
                    rx_state_ = INIT;  // ..and start again! (anyway)
                }
                //std::cout << "CHECK" << std::endl;
                break;

            default:
                rx_state_ = INIT;  // But it's somekind of error
                break;
        }
    }
}

int SerialCatchingDevice::serialWrite(byte *_data, size_t _length) {  
    // IO is currently non-blocking.
    int origflags = fcntl(serial_fd_, F_GETFL, 0);
    fcntl(serial_fd_, F_SETFL, origflags & ~O_NONBLOCK);
    int retval = ::write(serial_fd_, _data, _length);
    fcntl(serial_fd_, F_SETFL, origflags | O_NONBLOCK);

    if(retval == _length) {
        return retval;
    } else {
        snprintf(error_buffer_, ERROR_BUFFER_LENGTH, "[in function %s]::Write failed!", __FUNCTION__);
        throw std::runtime_error(error_buffer_);
    }
}
