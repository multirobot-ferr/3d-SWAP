#ifndef _MBZIRC_HALCLIENT_H_
#define _MBZIRC_HALCLIENT_H_

#include <grvc_quadrotor_hal/types.h>
#include <grvc_quadrotor_hal/server.h>

class HalClient {
public:
    HalClient(grvc::utils::ArgumentParser _args) {
        std::string uav_id = _args.getArgument<std::string>("uavId", "1");
        take_off_srv_  = new grvc::hal::Server::TakeOffService::Client       ("/mbzirc_" + uav_id + "/hal/take_off",  _args);
        land_srv_      = new grvc::hal::Server::LandService::Client          ("/mbzirc_" + uav_id + "/hal/land",      _args);
        waypoint_srv_  = new grvc::hal::Server::WaypointService::Client      ("/mbzirc_" + uav_id + "/hal/go_to_wp",  _args);
        pos_error_srv_ = new grvc::hal::Server::PositionErrorService::Client ("/mbzirc_" + uav_id + "/hal/pos_error", _args);
        abort_srv_     = new grvc::hal::Server::AbortService::Client         ("/mbzirc_" + uav_id + "/hal/abort",     _args);
    }

protected:
    grvc::hal::Server::TakeOffService::Client *take_off_srv_;
    grvc::hal::Server::LandService::Client *land_srv_;
    grvc::hal::Server::WaypointService::Client *waypoint_srv_;
    grvc::hal::Server::PositionErrorService::Client *pos_error_srv_;
    grvc::hal::Server::AbortService::Client *abort_srv_;
};

#endif  // _MBZIRC_HALCLIENT_H_
