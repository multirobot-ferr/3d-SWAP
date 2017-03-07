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
//------------------------------------------------------------------------------
#include <ros/ros.h>
#include <uav_state_machine/serial_catching_device.h>

int main(int _argc, char** _argv){
    std::cout << "Testing SerialCatchingDevice" << std::endl;
    ros::init(_argc, _argv, "test_node");
    ros::NodeHandle nh;
    SerialCatchingDevice catching_device(1, nh);
    std::cout << "Wait..."  << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(10));
    std::cout << "...for it!"  << std::endl;
    catching_device.setMagnetization(true);
    ros::spin();
    /*while (ros::ok()) {
        int cmd = getchar();
        bool magnetize = (cmd == '0') ? false : true;
        catching_device.setMagnetization(cmd);
        ros::spinOnce();
    }*/
}
