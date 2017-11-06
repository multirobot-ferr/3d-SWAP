/*
 * Copyright (c) 2017, University of Duisburg-Essen, swap-ferr
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of swap-ferr nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file measures.h
 * @date    30/10/17
 * @short: Node to show measures
 *
 * The following state machine is able to control the uav as the real one should
 * do. Moreover, it shares the necessary commands with swap to avoid collisions
 * when necessary.
 * This state machine can be used to test swap in the real robots before the full
 * integration to the system
 */


#ifndef MEASURES
#define MEASURES

#include <armadillo>
#include <stdlib.h> //rand

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <pid_controller.h>

// Uncomment this define if all the robots starts in (0,0)
//#define UAV_NOT_IN_ZERO_ZERO 1


// Constant values
const std::string pose_uav_topic = "/pose";

class Measures
{
    public:
        /**
         * @brief Default constructor of the class
         *
         * Request to ROS the necessary parameters
         * and connects to the publishers, subscribers
         * and services to control a single uav
         */
        Measures();
        /**
         * @brief Destructor to release the memory
         */
        ~Measures();

        /**
         * @brief Performs all necessary actions on the loop
         */
        void Loop();
        /**
         * @brief Executes the main loop of swap
         */
        void Spin();

        /**
         * @brief Executes the main loop of swap once
         */
        void SpinOnce();

    private:

        ros::NodeHandle nh_;                    //!< ROS Node handler
        ros::NodeHandle* pnh_;                  //!< Private node handler

        // Subscribers
        ros::Subscriber  pos_uav_sub_;          //!< Receives the position of the UAV



        // System variables
        int uav_id_ = -1;                       //!< Identification number of the current uav

        // Position of the UAV
        bool   pose_received_ = false;          //!< Tracks if the pose is already known
        double uav_x_, uav_y_, uav_z_, uav_yaw_;//!< Keeps the knowledge of the postion of the uav


        /* Callbacks for ROS */
        /**
         * @brief Callback for own pose estimation
         *
         * @param msg Position message from the grvc
         * @param uav_id Identifier for UAV
         */
        void PoseReceived(const geometry_msgs::PoseStamped::ConstPtr& uav_pose);




}; // class Measures

#endif // MEASURES
