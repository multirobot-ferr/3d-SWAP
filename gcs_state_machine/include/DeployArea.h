//
//  DeployPoint.hpp
//  DeploymentPoint
//
//  Created by Angel Rodriguez Castaño on 4/3/17.
//  Copyright © 2017 Angel Rodriguez Castaño. All rights reserved.
//

#ifndef MBZIRC_DEPLOY_AREA
#define MBZIRC_DEPLOY_AREA
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <gcs_state_machine/ApproachPoint.h>
#include <gcs_state_machine/DeployArea.h>
#include <grvc_utils/critical.h>

struct ApproachPointHandle {
    geometry_msgs::Point position;
    bool reserved;
    int uav_id;
    void print() {
        std::cout << "x = " << position.x << " y = " << position.y << \
        " uav_id = " << uav_id << (reserved? " reserved" : " free") << std::endl;
    }
};

class DeployAreaHandle {
public:
    DeployAreaHandle(const geometry_msgs::Point& _center, float _radius);
    bool ApproachPointSrvCallback(gcs_state_machine::ApproachPoint::Request &req, gcs_state_machine::ApproachPoint::Response &res);
    bool DeployAreaSrvCallback(gcs_state_machine::DeployArea::Request &req, gcs_state_machine::DeployArea::Response &res);
private:
    std::vector<ApproachPointHandle> approach_point_;
    ros::ServiceServer approach_point_service_;
    ros::ServiceServer deploy_area_service_;
    geometry_msgs::Point deploy_center_position_;
    grvc::utils::Critical<bool> deploy_area_reserved_;
    int deploy_area_owner_ = -1;
};

#endif  // MBZIRC_DEPLOY_AREA
