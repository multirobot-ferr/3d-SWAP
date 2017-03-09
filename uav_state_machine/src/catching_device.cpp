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
#include <uav_state_machine/catching_device.h>
#include <uav_state_machine/serial_catching_device.h>
#include <uav_state_machine/gazebo_catching_device.h>

CatchingDevice* CatchingDevice::createCatchingDevice(unsigned int _uav_id, ros::NodeHandle& _nh) {
    CatchingDevice* implementation = nullptr;
    // Return an implementation based on rosparam 'mavros_spawn/(_uav_id)/mode'
    std::string param_name = "mavros_spawn/" + std::to_string(_uav_id) + "/mode";
    std::string mode;
    _nh.param<std::string>(param_name, mode, "real");  // Get param, default is 'real' mode
    if (mode == "real") {
        implementation = new SerialCatchingDevice(_uav_id, _nh);
    } else if (mode == "sitl") {
        implementation = new GazeboCatchingDevice(_uav_id, _nh);
    }
    return implementation;
}
