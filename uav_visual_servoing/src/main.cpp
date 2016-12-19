//
//
//
//
//
//

#include <grvc_utils/argument_parser.h>
#include <grvc_quadrotor_hal/types.h>
#include <grvc_quadrotor_uav/server.h>

#include <thread>
#include <chrono>

int main(int _argc, char** _argv){
    // Init services.
    grvc::utils::ArgumentParser args(_argc, _argv);
    grvc::uav::Server::TakeOffService::Client take_off_srv("/quad1/uav/take_off", args);
    grvc::uav::Server::WaypointService::Client waypoint_srv("/quad1/uav/go_to_wp", args);
    grvc::uav::Server::PathService::Client path_srv("/quad1/uav/path", args);
    grvc::uav::Server::LandService::Client land_srv("/quad1/uav/land", args);
    std::this_thread::sleep_for(std::chrono::seconds(5));

    bool enableVisualServoing = true;

    while(enableVisualServoing){

    }
}
