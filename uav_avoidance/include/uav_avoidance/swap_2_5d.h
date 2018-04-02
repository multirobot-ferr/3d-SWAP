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
 *
 *
 */

/**
 * @file swap_2_5.h
 * @author Eduardo Ferrera
 * @version 0.4
 * @date    12/3/17
 *
 * @short: Adapts the avoidace code "SWAP" to the GRVC enviroment for the mbzirc challenge.
 *
 * This node is designed to work in parallel with a state_machine. It communicates with the state machine throw the
 * "collision_warning", the "wished_movement_direction" and the "avoid_movement_direction" topics.
 * Swap_2_5d will connect to all the robots of the system and warns through "collision_warning" to the state machine
 * when two or more uavs are too close and can collide. When that happens, Swap_2_5d will publish over
 * "avoid_movement_direction" an avoidance direction for the uav.
 * It also receives from the state machine "wished_movement_direction", a vector that indicates where does the uav wants
 * to go (e.g: where is placed the next goal, or in wich direction does it wants to move).
 *
 */

#ifndef SWAP_2_5D_H
#define SWAP_2_5D_H

#include <swap.h>                    // base class
#include <vector>
#include <map>
#include <sstream>

// message need
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_ros/point_cloud.h>
#include <std_srvs/SetBool.h>




// Fixing some problems
// If the simulator makes all uavs start in (0,0), this define should be uncommented
//#define UAV_NOT_IN_ZERO_ZERO 1

#ifdef UAV_NOT_IN_ZERO_ZERO
// values extracted from the simulator:
                        //   x      y   yaw
const arma::mat UAV_ZZ = { {-28.0, 34.0, 0.0 },
                           {-20.0, 30.0, 0.0 },
                           {-28.0, 26.0, 0.0 } };
#endif

// Constant values
const std::string pose_uav_topic = "/pose";

class Swap_2_5d:  public avoid::Swap
{
    public:
        
        bool experiment_started_ = false; 
        /**
         * @brief Default constructor of the class
         *
         * Request to ROS the necessary variables from the parameters and connects
         * the necessary publishers and subscribers.
         */
        Swap_2_5d();

        /**
         * @brief Destructor to release the memory
         */
        virtual ~Swap_2_5d();

        /**
         * @brief Controls if all parameters are well initialized
         *
         * @param[out] why_not returns why the system is not well initialized.
         * @return Whether the system is ready or not.
         */
        bool IsReady(std::string& why);

        /**
         * @brief Executes the main loop of swap
         */
        void Spin();

        /**
         * @brief Executes the main loop of swap once
         */
        void SpinOnce();

    private:
        ros::NodeHandle nh_;            //!< ROS Node handler
        ros::NodeHandle* pnh_;          //!< Private node handler


        // Subscribers
        std::vector<ros::Subscriber> pos_all_uav_sub_;  //!< Receives the positions of all UAVs
        ros::Subscriber wished_mov_dir_sub_;            //!< Receives the direction where the uav whants to go
        ros::Subscriber laser_sub_;                     //!< Receives laser data
        ros::Subscriber pointcloud_sub_;                //!< Receives lidar data
        ros::ServiceServer wait_for_start_;           //!< Service that makes the UAV wait on the Goal point 1


        // Publishers
        ros::Publisher confl_warning_pub_;              //!< Publisher to determine if there is a possible collision or not
        ros::Publisher avoid_mov_dir_pub_;             //!< Publish the direction where the uav has to go to avoid a conflict
        ros::Publisher xyz_pub_;                       //!< Publish the xyz position from the lidar
        ros::Publisher marker_pub;                     //!< Publish a marker to visualizate robot orientation
        // System variables
        bool initialization_error_ = false;             //!< Flags to track possible errors in initialization
        double spin_sleep_ = 0.1;
        bool hard_debug_=false;                         //!< Time that the system will sleep between iterations of the main loop
        int uav_id_ = -1;                               //!< Identification number of the current uav
        int n_uavs_;                                    //!< Number of UAV involved
        std::vector<int> uav_ids_;                      //!< IDs of UAV involved
        double uav_vector_speed_;                       //!< Modulus of the vector sent to the uav for avoidance
        double uav_safety_radius_ = -1.0;               //!< Safety radius of each uav
	    double dz_min_;	    		   		            //!< max z-distance to swap. It is a parameter
        double dz_range_;
        pcl::PointCloud<pcl::PointXYZ> cloud_xyz_;      //!< Lidar information to publish


        //Noise
        std::default_random_engine generator_;
        std::normal_distribution<double> distribution_;

        // Position of the UAV
        bool   pose_received_ = false;                  //!< Tracks if the uav knows its position
        double uav_x_, uav_y_, uav_z_, uav_yaw_;        //!< Position of the current uav
        bool z_swap_=true;			    		        //!< Flag to swap depends on z-distance between uavs

        // START ----- IMPORTANT ----- START
        // uav_yaw_ is the direction where the robot wants to go
        // with respect to the x axis, not his orientation
        double uav_wished_yaw_map_;
        // END ------- IMPORTANT ------- END


        // Swap variables
        double v_ref_;      //!< Since swap has no control of the speed, it can be ignored
        double yaw_ref_;    //!< Orientation with respect to the nord that the uav should take to avoid a conflict
        geometry_msgs::Vector3 avoid_mov_direction_;    //!< Message where the avoidance direction will be published
	
        bool start_experiment_ = false;
        // Debuging parameters
        std::ofstream log2mat_;
        std::vector<double> values2log_;    // Defined here to avoid multiple allocations of memory
        double bracking_distance_, positioning_error_, gamma_offset_;
        double *pos_all;
        std::map<int,int> log_pos_map_;


        /* Callbacks for ROS */
        /**
         * @brief Callback for own pose estimatimation
         *
         * @param uav_pose Position message from the grvc
         * @param uav_id Identifier from UAV
         */
        void PoseReceived(const geometry_msgs::PoseStamped::ConstPtr& uav_pose, const int uav_id);

        /**
         * @brief Callback for Lidar date
         * 
         * @param cloud_msg Point cloud message from Lidar
         */
        void CloudCallback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

        /**
         * @brief Callback for laser 2D date
         * 
         * @param scan Laser 2D message from Hokuyo
         */
        void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

        /**
         * @brief Callback for the direction of movement of the uav
         *
         * @param movement_direction_uav with respect to the map
         */
        void WishedMovDirectionCallback(const geometry_msgs::Vector3::ConstPtr& movement_direction_uav);

        /* Internal publishers */
        /**
         * @brief Publishes if there is a possible collision to avoid and the direction to take
         */
        void RequestControlPub(bool request_control);

        /**
         * @brief Saves a single line of a log file from the simulation that can be read in matlab
         *
         * Does not activate if the parameter "debug/file_path" is not set
         */
        void FillLogFile();

        /**
         * @brief Prepares a logger for matlab
         */
        void Log2MatlabInit( const std::string file_path);

        /**
         * @brief Creates a log file that can be read in matlab
         * @param vector of values that will be saved
         */
        void Log2Matlab( std::vector<double>& values);

        /**
         * @brief Utility function. 
         * 
         * Utility function. Publish a market to visualizate robot orientation 
         */
        void PolarObstacleMarker();

        bool StartServiceCb(std_srvs::SetBool::Request& allowed2move, std_srvs::SetBool::Response& ok);

}; // class SwapRos


#endif // SWAP_2_5D_H
