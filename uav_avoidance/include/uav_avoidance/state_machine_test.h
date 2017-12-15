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
 * @file state_machine_test.h
 * @author Eduardo Ferrera
 * @version 0.8
 * @date    12/3/17
 * @short: Emulates a state machine that makes the uav fly to different goals
 *
 * The following state machine is able to control the uav as the real one should
 * do. Moreover, it shares the necessary commands with swap to avoid collisions
 * when necessary.
 * This state machine can be used to test swap in the real robots before the full
 * integration to the system
 */


#ifndef STATE_MACHINE_TEST
#define STATE_MACHINE_TEST

#include <armadillo>
#include <stdlib.h> //rand

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/SetBool.h>
#include <uav_avoidance/pid_controller.h>  
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>       

// Uncomment this define if all the robots starts in (0,0)
//#define UAV_NOT_IN_ZERO_ZERO 1

#ifdef UAV_NOT_IN_ZERO_ZERO
// values extracted from the simulator:
                        //   x      y   yaw
/*const arma::mat UAV_ZZ = { {-28.0, 34.0, 0.0 },
                           {-20.0, 30.0, 0.0 },
                           {-28.0, 26.0, 0.0 } }; */
#endif

// Constant values
const std::string pose_uav_topic = "/pose";
const std::string takeoff_service= "/take_off";
const std::string speed_service  = "/set_velocity";
const std::string way_point_service  = "/go_to_waypoint";
const std::string pos_err_service = "/set_position_error";
const std::string land_service   = "/land";

/**
 * Main class of the state machine
 */ 
class StateMachine
{
    public:
        /**
         * @brief Default constructor of the class
         *
         * Request to ROS the necessary parameters
         * and connects to the publishers, subscribers
         * and services to control a single uav
         */
        StateMachine();

        /**
         * @brief Destructor to release the memory
         */
        ~StateMachine();

        /**
         * @brief Performs all necessary actions on the loop
         */
        void Loop();

        /**
         * @brief Lands the uav
         */
        void Land();

        /**
         * @brief Forces a land of the UAV out of the node
         */
        void ForcedLand();

        /**
         * @brief Returns true while the system still flying
         */
        bool Running();


    private:


        ros::NodeHandle nh_;                    //!< ROS Node handler
        ros::NodeHandle pnh_{"~"};                  //!< Private node handler

        // Subscribers
        ros::Subscriber  pos_uav_sub_;          //!< Receives the position of the UAV

        // Services
        ros::ServiceServer wait_for_start_;           //!< Service that makes the UAV wait on the Goal point 1
        bool keep_moving_ = false;              //!< Allows the UAV to move on from the Goal point 1
        bool landed_ = false;                     

    // ###########  Communication with SWAP  ########### //
        ros::Subscriber  confl_warning_sub_;    //!< Receives warnings from SWAP
        bool confl_warning_ = false;            //!< Flag to know if there is a conflict to avoid
        ros::Publisher   wished_mov_dir_pub_;   //!< Sends the direction of movement that the uav wants to take (respect to the map)
        ros::Subscriber  avoid_mov_dir_sub_;    //!< Requested direction to avoid a conflict
        geometry_msgs::Vector3 wished_direction_uav_; //!< Movement direction with respect to the map (where the uav wants to go)
        geometry_msgs::Vector3 avoid_mov_direction_uav_; //!< Movement direction requested from swap
    // ###########  #######################  ########### //

        //pid variables
        grvc::utils::PidROS xv_pid_{"xv_pid"};
        grvc::utils::PidROS yv_pid_{"yv_pid"};
        grvc::utils::PidROS zv_pid_{"zv_pid"};
        grvc::utils::PidROS yawv_pid_{"yaw_pid"};



        // System variables
        bool experiment_done = false;           
        bool initialization_error = false;      //!< Tracks possible initialization errors
        int uav_id_ = -1;                       //!< Identification number of the current uav
        bool yaw_on_;                   //!< movement depend on laser 3D is activated
        bool game_frame_;
        // Position of the UAV
        bool   pose_received_ = false;          //!< Tracks if the pose is already known
        double uav_x_, uav_y_, uav_z_, uav_yaw_;//!< Keeps the knowledge of the postion of the uav


        // UAV references
        double z_ref_;
        double d_goal_;
        double v_ref_ = 1.5;


        //Preparing to command the UAV
        ros::ServiceClient takeOff_srv_;
        ros::ServiceClient way_point_srv_;
        ros::ServiceClient speed_srv_;
        ros::ServiceClient pos_err_srv_;
        ros::ServiceClient land_srv_;
        double dist_between_uav_z_ = 0.0;

        //Waypoints to debug the system
        unsigned wp_idx_ = 0;    //  x    y   yaw
        arma::mat way_points_ ={ {-28.0, +20.0, 0.0    },
                                 {-28.0, +40.0, 0.0    },
                                 {-31.0, +40.0, M_PI   },
                                 {-25.0, +26.0,-M_PI   },
                                 {-31.0, +26.0, M_PI/2 },
                                 {-31.0, +20.0,-M_PI/4 },
                                 {-25.0, +40.0, M_PI/16} };

                                 ros::Time t = ros::Time::now();



        /* Callbacks for ROS */
        /**
         * @brief Callback for own pose estimation
         *
         * @param msg Position message from the grvc
         * @param uav_id Identifier for UAV
         */
        void PoseReceived(const geometry_msgs::PoseStamped::ConstPtr& uav_pose);

        /**
         * @brief Allows the UAV to move from the position 1 to the next one
         * 
         * @param 
         */
        bool StartServiceCb(std_srvs::SetBool::Request& allowed2move, std_srvs::SetBool::Response& ok);

    // ###########  Communication with SWAP  ########### //
        /**
         * @brief Callback from SWAP that informs to the state machine of a conflict
         *
         * @param msg Boolean equal to true if there is a collision risk
         */
        void WarningCallback(const std_msgs::Bool::ConstPtr& collision_warning);
    // ###########  #######################  ########### //

    // ###########  Communication with SWAP  ########### //
        /**
         * @brief Informs to the state machine of the direction to take in case of a conflict
         * @param avoidance_direction direction to take in case of conflict
         */
        void AvoidMovementCallback(const geometry_msgs::Vector3::ConstPtr& avoidance_direction);
    // ###########  #######################  ########### //

        /* Internal publishers */
    // ###########  Communication with SWAP  ########### //
        /**
         * @brief Computes the necessary control actions for the robot and publishes them
         *
         * Makes the robot move to the next goal unless the collision avoidance system informs
         * from a possible collision. On that case, follows the avoidance reference
         */
        void PublishPosErr();
    // ###########  #######################  ########### //

        /**
         * @brief Publish a position error on the controller of the uav
         * @param xe error in x
         * @param ye error in y
         * @param ze error in z
         */
        void PublishGRVCPosErr(const double xe, const double ye, const double ze);

        /**
         * @brief Publishes a speed on the grvc controler
         * @param vx speed in the x coordinate
         * @param vy speed in the y coordinate
         * @param vz speed in the z coordinate
         */
        void PublishGRVCCmdVel(const double vx, const double vy, const double vz = 0.0, const double yaw_rate = 0.0);

        /**
         * @brief Publishes a goal on the grvc controler
         * @param x coordinate x of the goal
         * @param y coordinate y of the goal
         * @param z coordinate z of the goal
         * @param yaw coordinate yaw of the goal
         */
        void PublishGRVCgoal(const double x, const double y, const double z, const double yaw);

        /**
         * @brief Takes off the quadrotor
         * @return Success of the operation
         */
        void TakeOff();

        /**
         * @brief If the robot is close to a waypoint, it actualizes to the next waypoint
         */
        void UpdateWayPoints();

        /**
         *  Pid implementation
         */

        double pid(double error, double dt);

        /**
         *
         */
        double ScaleAngle(double angle);

        /**
         * utility function to transform waypoints
         */
        
        arma::mat gameToMap(arma::mat wp_game, double yaw_rot);
        

}; // class StateMachine

#endif // STATE_MACHINE_TEST
