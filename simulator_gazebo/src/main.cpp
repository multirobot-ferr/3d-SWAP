//----------------------------------------------------------------------------------------------------------------------
// GRVC Quadrotor HAL sample
//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
// 
// Copyright (c) 2016 GRVC University of Seville
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//----------------------------------------------------------------------------------------------------------------------
#include <grvc_com/publisher.h>
#include <grvc_quadrotor_hal/server.h>
#include <cstdint>
#include <thread>
#include <vector>
#include <string>
#include <thread>

using namespace grvc::com;
using namespace grvc::hal;

void updateQuadrotor(const std::string & quad_name, const std::vector<Vec3>&wplist, int _argc, char **_argv)
{
    // Use this to send waypoints to hal
    Publisher* wpPub = new Publisher("hal_sample", (quad_name + "/hal/go_to_wp").c_str(), _argc, _argv);
    Publisher* takeOffPub  = new Publisher("hal_sample", (quad_name + "/hal/take_off").c_str(), _argc, _argv);
    Publisher* landPub  = new Publisher("hal_sample", (quad_name + "/hal/land").c_str(), _argc, _argv);
    std::string curState;
    new Subscriber<std::string>("hal_sample", (quad_name + "/hal/state").c_str(), _argc, _argv, [&](const std::string& _str) {
        curState = _str;
    });


   while(curState != "finished")
    {}
   takeOffPub->publish(0.5);
   std::this_thread::sleep_for(std::chrono::seconds(3));
   for (size_t i = 0; i < wplist.size(); ++i) {
        wpPub->publish(wplist[i]);
        std::this_thread::sleep_for(std::chrono::seconds(4));
   }
    landPub->publish();
    std::this_thread::sleep_for(std::chrono::seconds(3));
}

int main(int _argc, char** _argv) {


    std::vector<Vec3>wplist_1 = {{60.0, -25.0, 20},{30.0, -20.0, 20},{0.0, -20.0, 20},{-30.0, -20.0, 20},{-50.0, -20.0, 20},{-50.0, 0.0, 20},{-50.0, 20.0, 20},{-30.0, 20.0, 20},{0.0, 20.0, 20},{30.0, 20.0, 20},{-30.0, 20.0, 20},{50.0, 20.0, 20}};
    //updateQuadrotor("/quad1", wplist_1, _argc, _argv);

    std::thread quad1(updateQuadrotor, "/quad1", wplist_1, _argc, _argv);

    std::vector<Vec3>wplist_2 = {{60, -15.0, 16},{30, -5.0, 16},{0.0, 0.0, 16},{-30, 10.0, 16},{-50, 20.0, 16},{0, 20.0, 16},{10, 0.0, 16},{30, -5.0, 16},{0.0, 0.0, 16},{-30, 10.0, 16},{-50, 20.0, 16},{0, 20.0, 16},{10, 0.0, 16}};
    //updateQuadrotor("/quad2", wplist_2, _argc, _argv);

    std::thread quad2(updateQuadrotor, "/quad2", wplist_2, _argc, _argv);

    std::vector<Vec3>wplist_3 = {{60.0, -10.0, 18}, {55.0, -10.0, 18},{45.0, -5.0, 18},{35.0, -10.0, 18},{25.0, -5, 18},{20.0, 0.0, 18},{20.0, 5.0, 18},{10.0, -5.0, 18},{0.0, 5.0, 18},{-10.0, 10.0, 18},{-15.0, 10.0, 18},{-25.0, 0.0, 18},{-10.0, -10.0, 18.0}};
    //updateQuadrotor("/quad2", wplist_2, _argc, _argv);

    std::thread quad3(updateQuadrotor, "/quad3", wplist_3, _argc, _argv);

    //std::vector<Vec3>wplist_4 = {{10.0, 10.0, 0.05}, {-10.0, 10.0, 0.05}, {-10.0, -10.0, 0.05}, {10.0, -10.0, 0.05}, {10.0, 10.0, 0.05}, {-10.0, 10.0, 0.05}, {-10.0, -10.0, 0.05}, {10.0, -10.0, 0.05}};
   // std::thread redbox(updateQuadrotor, "/Object_Red_Box2", wplist_4, _argc, _argv);

    quad1.join();
    quad2.join();
    quad3.join();

	return 0;
}
