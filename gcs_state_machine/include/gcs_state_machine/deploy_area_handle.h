//------------------------------------------------------------------------------
// GRVC MBZIRC 
//------------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2017 GRVC University of Seville
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
#ifndef MBZIRC_DEPLOY_AREA_HANDLE_H
#define MBZIRC_DEPLOY_AREA_HANDLE_H
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <gcs_state_machine/ApproachPoint.h>
#include <gcs_state_machine/DeployArea.h>
#include <handy_tools/critical.h>

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

#endif  // MBZIRC_DEPLOY_AREA_HANDLE_H
