/*
 * Node for move object
 * Author: Luis Manuel Ramirez de la Cova
 * Date: 30-05-2016
 * Modificate: 31-05-2016
 * Organization: University of Seville, GRVC
 * Description:
 */

#include <grvc_com/publisher.h>
#include <grvc_quadrotor_hal/server.h>
#include <cstdint>
#include <thread>
#include <vector>
#include <string>
#include <thread>

using namespace grvc::com;
using namespace grvc::hal;

void updateObject(const std::string & object_name, const std::vector<Vec3>&wplist, int _argc, char **_argv)
{
    Publisher* wpPub = new Publisher("hal_sample", (object_name + "/hal/go_to_wp").c_str(), _argc, _argv);
    std::string curState;
    new Subscriber<std::string>("hal_sample", (object_name + "/hal/state").c_str(), _argc, _argv, [&](const std::string& _str) {
        curState = _str;
    });

    while(curState != "finished")
    {}
    std::this_thread::sleep_for(std::chrono::seconds(3));
    for (size_t i= 0; i < wplist.size(); i++)
    {
        wpPub->publish(wplist[i]);
        std::this_thread::sleep_for(std::chrono::seconds(4));
    }
}


int main(int _argc, char** _argv)
{
    std::vector<Vec3>wplist_1 = {{-50.0, -20.0, 0.07},{50.0, -20.0, 0.07},{50.0, 20.0, 0.07},{-50.0, 20.0, 0.07}, {-50.0, -20.0, 0.07},{50.0, -20.0, 0.07},{50.0, 20.0, 0.07},{-50.0, 20.0, 0.07}, {-50.0, -20.0, 0.07},{50.0, -20.0, 0.07},{50.0, 20.0, 0.07},{-50.0, 20.0, 0.07}};
    //updateObject("/object_blue_cylinder_mov", wplist_1, _argc, _argv);

    std::thread object_blue_cylinder_mov(updateObject, "/object_blue_cylinder_mov", wplist_1, _argc, _argv);

    object_blue_cylinder_mov.join();

    return 0;

}
