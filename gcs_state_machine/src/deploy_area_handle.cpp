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
#include <gcs_state_machine/deploy_area_handle.h>
#include <iostream>
#include <math.h>

using namespace gcs_state_machine;

DeployAreaHandle::DeployAreaHandle(const geometry_msgs::Point& _center, float _radius) {
    const int num_points = 6;
    float inc = (float)(2*M_PI/num_points);
    float ang = 0.0;   
    for (size_t i = 0; i < num_points; i++) {
        ApproachPointHandle point;
        point.position.x = _center.x + _radius * cos(ang);
        point.position.y = _center.y + _radius * sin(ang);
        point.reserved = false;
        point.uav_id = -1;
        approach_point_.push_back(point);
        ang += inc;
    }

    deploy_area_reserved_.set(false);
    deploy_center_position_ = _center;

    // Advertise services:
    ros::NodeHandle nh;
    approach_point_service_ = nh.advertiseService("mbzirc_gcs/approach_point", \
    &DeployAreaHandle::ApproachPointSrvCallback, this);
    deploy_area_service_ = nh.advertiseService("mbzirc_gcs/deploy_area", \
    &DeployAreaHandle::DeployAreaSrvCallback, this);
}

bool DeployAreaHandle::ApproachPointSrvCallback(ApproachPoint::Request &req, ApproachPoint::Response &res) {
    switch (req.question) {
        case ApproachPoint::Request::RESERVE_APPROACH_POINT: {
            float dist_min = 30000.0;
            int index = -1;
            for (size_t i = 0; i < approach_point_.size(); i++) {
                float ex = req.uav_position.x - approach_point_[i].position.x;
                float ey = req.uav_position.y - approach_point_[i].position.y;
                float dist = ex*ex + ey*ey;
                if ((dist < dist_min) && (!approach_point_[i].reserved)) {
                    dist_min = dist;
                    index = i;
                }
            }
            if (index >= 0) {
                approach_point_[index].uav_id = req.uav_id;
                approach_point_[index].reserved = true;
                res.approach_position.x = approach_point_[index].position.x;
                res.approach_position.y = approach_point_[index].position.y;
                res.answer = ApproachPoint::Response::OK;
            } else {
                res.answer = ApproachPoint::Response::WAIT;  // But it's an error...
                // ... someone forgot to free, there are 6 approach points for 3 uavs!
            }
            break;
        }

        case ApproachPoint::Request::FREE_APPROACH_POINT: {
            for (size_t i = 0; i < approach_point_.size(); i++) {
                if (approach_point_[i].uav_id == req.uav_id) {
                    approach_point_[i].reserved = false;
                }
            }
            // And no info in response expected...
            break;
        }

        default:
            std::cerr << "Approach point service callback: not expected question!"  << std::endl;  // wtf?
            break;
    }
    return true;  // TODO: Use return value instead of OK / WAIT?
}

bool DeployAreaHandle::DeployAreaSrvCallback(DeployArea::Request &req, DeployArea::Response &res) {
    switch (req.question) {
        case DeployArea::Request::FREE_DEPLOY_AREA:
            if (deploy_area_owner_ == req.uav_id) {
                deploy_area_reserved_.set(false);
                res.answer = DeployArea::Response::OK;
            } else {
                // Only the owner can free...
                res.answer = DeployArea::Response::WAIT;
            }
            break;
        case DeployArea::Request::RESERVE_DEPLOY_AREA:
            // Set reserved to true if it equals false
            if (deploy_area_reserved_.setIfDataEquals(true, false)) {
                deploy_area_owner_ = req.uav_id;
                res.deploy_position = deploy_center_position_;
                res.answer = DeployArea::Response::OK;
            } else {
                res.answer = DeployArea::Response::WAIT;
            }
            break;
        default:
            std::cerr << "Deploy area service callback: not expected question!" << std::endl;
            break;
    }
    return true;
}
