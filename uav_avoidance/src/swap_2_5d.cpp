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
 * @file swap_2_5.cpp
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

#include <uav_avoidance/swap_2_5d.h>
#include <std_msgs/Bool.h>
#include <tf2/utils.h>
#include <functional>


int main(int argc, char **argv) {
    // name remapping
    ros::init(argc, argv, "swap_2_d");

    Swap_2_5d swap;

    std::string why;
    if (!swap.IsReady(why))
    {
        ROS_FATAL("%s", why.c_str());
        ROS_FATAL("SWAP: Parameters are not properly configured, closing swap");
    }
    else
    {
        swap.Spin();
    }

    return 0;
}

/**
 * Default constructor of the class
 */
Swap_2_5d::Swap_2_5d()
{
    // Preparing private acquisition of parameters
    pnh_ = new ros::NodeHandle("~");

    // Sleeping some random time to not collapse the system due to a big wake up
    // Waiting a time between 1 and 2 seconds
    // TODO Seems to have allways the same number -> 1.4
    double sleep_time = 1.0 + (rand() % 10 + 1)/10.0;
    ROS_INFO("SWAP: Waiting %f seconds before start", sleep_time);
    ros::Duration( sleep_time).sleep();

    // Getting the uav_ids
    n_uavs_ = 0;
    if (!pnh_->getParam("uav_ids", uav_ids_)) {
        initialization_error_ = true;
        ROS_FATAL("SWAP: uav_ids are not set. Closing the avoidance system");
    }
    else
        n_uavs_ = uav_ids_.size();

    // Getting the uav_id
    if (!pnh_->getParam("uav_id", uav_id_)) {
        initialization_error_ = true;
        ROS_FATAL("SWAP: uav_id is not set. Closing the avoidance system");
    }

    // getting max z-distance between uavs to swap
    pnh_->param<double>("swap/dz_min", dz_min_,1.0);


    // getting max z-distance between uavs to swap
    pnh_->param<double>("swap/dz_range", dz_range_,1.0);


    // Getting the sleeping time of the loop
    pnh_->param<double>("spin_sleep", spin_sleep_, 0.2);

    // Getting the normal speed of an uav
    pnh_->param<double>("uav_vector_speed", uav_vector_speed_, 1.0);

    // Getting swap parameters
    int granularity;
    if (pnh_->getParam("swap/granularity", granularity))
    {
        SetGranularity(granularity);
    }

    int num_measurements;
    if (pnh_->getParam("swap/num_measurements", num_measurements))
    {
        SetMeasurementsPerDirection(num_measurements);
    }

    double multi_input_pond;
    if (pnh_->getParam("swap/multi_input_pond", multi_input_pond))
    {
        SetMultiInputPond(multi_input_pond);
    }

    if (pnh_->getParam("swap/safety_radius", uav_safety_radius_))
    {
        SetSafetyRegion(uav_safety_radius_);         // All UAV are equal right now
    }

    if (pnh_->getParam("swap/braking_distance", bracking_distance_))
    {
        SetBrackingDistance(bracking_distance_);
    }

    SetLocalMeasurementError(0.0);     // The system has not a laser ranger
    if (pnh_->getParam("swap/positioning_error", positioning_error_))
    {
        SetGlobalMeasurementError(positioning_error_);
    }

    if (pnh_->getParam("swap/gamma_offset", gamma_offset_))
    {
        SetGammaOffset(gamma_offset_);
    }

    SetSmoothFactor(0); // The system has no laser ranger. It makes no sense to smooth the measurements
    SetHolonomicRobot(true);

    double rotation_ctrl_p;
    if (pnh_->getParam("swap/rotation_ctrl_p", rotation_ctrl_p))
    {
        SetRotCtrlP(rotation_ctrl_p);
    }

    // Specific namespace for UAL    
    std::string ual_ns;
    if (!pnh_->getParam("ual_namespace", ual_ns))
        ual_ns = "";

    // Subscribing to the position of all UAVs
    for (int n_uav = 0; n_uav < n_uavs_; n_uav++) {
        std::string uav_topic_name = "/" + ual_ns + "ual_" + std::to_string(uav_ids_[n_uav]) + pose_uav_topic.c_str();
        pos_all_uav_sub_.push_back(nh_.subscribe<geometry_msgs::PoseStamped>(uav_topic_name.c_str(), 1, std::bind(&Swap_2_5d::PoseReceived, this, std::placeholders::_1, uav_ids_[n_uav]) ));
    }

    // Information for the state machine
    confl_warning_pub_  = nh_.advertise<std_msgs::Bool>("collision_warning", 1, true);
    wished_mov_dir_sub_ = nh_.subscribe( "wished_movement_direction",1 , &Swap_2_5d::WishedMovDirectionCallback, this);
    avoid_mov_dir_pub_  = nh_.advertise<geometry_msgs::Vector3>("avoid_movement_direction", 1, true);
    laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/mbzirc_1/front_laser/scan",10,&Swap_2_5d::LaserCallback,this);

    // Meant to debug the system
    std::string file_path;
    if (pnh_->getParam("debug/file_path", file_path))
    {
        Log2MatlabInit( file_path);
    }

    if (!initialization_error_)
    {
        ROS_INFO("SwapROS: Init complete");
    }
}

/**
 * Destructor to release the memory
 */
Swap_2_5d::~Swap_2_5d()
{
    // Closing debug log
    if (log2mat_)
    {
        log2mat_.close();
        delete pos_all;
    }

    // Releasing the memory
    delete pnh_;
}

/**
 * Controls if all parameters are well initialized
 */
bool Swap_2_5d::IsReady(  std::string& why_not)
{
    if (!Swap::IsReady(why_not))
    {
        return false;
    }
    if (initialization_error_)
    {
        why_not = "There was an initialization error on SWAP";
        return false;
    }
    return true;
}

/**
 * Executes the main loop of swap
 */
void Swap_2_5d::Spin()
{
    while (ros::ok())
    {
        SpinOnce();

        // Sleeping to save time
       ros::Duration(spin_sleep_).sleep();
    }
}

/**
 * Executes the main loop of swap once
 */
void Swap_2_5d::SpinOnce()
{
    // Forgetting old information
    //TODO Create a watchdog here to see if the information is comming or not
    NewMeasurementSetReceived();    //This forgets old information (slowly)

    // Getting fresh information (updating positions and so on)
    ros::spinOnce();

    // Checking for conflicts and finding solutions
    CollisionAvoidance(v_ref_, yaw_ref_, uav_vector_speed_);

    // If active, save in a log all information
    FillLogFile();

    // Communicating with the state machine
    std::string state;
    switch (GetMachineState(state))
    {
        case state_orientation::FREE:
            RequestControlPub(false);
            break;

        default:
            // Requesting the control of the uav and commanding it
          // if (z_swap_==true){

            RequestControlPub(true);


    }

    // Showing to the user what is happening
  //  ROS_INFO("Swap: %s", state.c_str());
    if (GetMachineStateChanges(state))
    {
        ROS_INFO("Swap: %s", state.c_str());
    }
}

/**
  * Callback for laser
  */

void Swap_2_5d::LaserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{


   for (unsigned id_laser = 0; id_laser < scan->ranges.size(); ++id_laser)
       {
           double angle = scan->angle_min + double(id_laser)*scan->angle_increment;
           SetNewLocalMeasurement( scan->ranges[id_laser]- positioning_error_ , angle, false);
       }

}

/**
 * Callback for own pose estimatimation
 */
void Swap_2_5d::PoseReceived(const geometry_msgs::PoseStamped::ConstPtr& uav_pose, const int uav_id)
{
    
    tf2::Quaternion q3(uav_pose->pose.orientation.x,
                       uav_pose->pose.orientation.y,
                       uav_pose->pose.orientation.z,
                       uav_pose->pose.orientation.w);
    q3.normalize();     //Avoids a warning

    std::vector<double> offset(3,0.0);
    #ifdef UAV_NOT_IN_ZERO_ZERO
    for (int coordinate = 0; coordinate < 3; ++coordinate)
    {
        offset[coordinate] = UAV_ZZ( uav_id-1, coordinate );
    }
    #endif
    double x   = offset[0] + uav_pose->pose.position.x;
    double y   = offset[1] + uav_pose->pose.position.y;
    double z   = offset[2] + uav_pose->pose.position.z;
    double yaw = tf2::getYaw(q3);

    if (uav_id == uav_id_)
    {
        // We are reading the position of the robot that owns the avoidance system
        pose_received_ = true;
        uav_x_   = x;
        uav_y_   = y;
        uav_z_   = z;
        uav_yaw_ = yaw;

        //ROS_INFO("Pose of uav %d %.2f, %.2f, %.2f, ||  %.2f", uav_id, x, y, z, yaw);
    }
    else if (pose_received_)
    {
        // The robot knows where it is and where an other robot is located.
        SetNewGlobalMeasurement(uav_x_, uav_y_, uav_z_, 0.0,  // The 0.0 makes it always look to the nord (even if not)
                                x, y, z, uav_safety_radius_, true);


    }

    // If necessary, save the positions on the log
    if (pos_all)
    {
        int pos = log_pos_map_[uav_id];

        pos_all[pos*3 + 0] = x;
        pos_all[pos*3 + 1] = y;
        pos_all[pos*3 + 2] = z;
    }
}

/**
 * Callback for the direction of movement of the uav
 */
void Swap_2_5d::WishedMovDirectionCallback(const geometry_msgs::Vector3::ConstPtr& wished_movement_direction_uav)
{
    uav_wished_yaw_map_ = atan2(wished_movement_direction_uav->y,
                                wished_movement_direction_uav->x);
    // The goal is far away to avoid swap to think that is close .
    // If that happens, swap will command to the system to brake
    SetGoal( 100.00, uav_wished_yaw_map_);
}

/**
 * Publishes if there is a possible collision to avoid and the direction to take
 */
void Swap_2_5d::RequestControlPub(bool request_control)
{
    std_msgs::Bool request_ctrl;
    request_ctrl.data = request_control;
    confl_warning_pub_.publish(request_ctrl);

    if (request_control)
    {
        avoid_mov_direction_.x = v_ref_*cos(yaw_ref_);
        avoid_mov_direction_.y = v_ref_*sin(yaw_ref_);
        avoid_mov_direction_.z = 0.0;   // The state machine should ignore this value.
        avoid_mov_dir_pub_.publish(avoid_mov_direction_);

    }
}

/**
 * Saves a single line of a log file from the simulation that can be read in matlab
 */
void Swap_2_5d::FillLogFile()
{
    if (log2mat_)
    {
        values2log_.clear();

        //TODO: Add the ros time

        for (auto id = 0; id < n_uavs_; ++id )
        {
            int pos = log_pos_map_[uav_ids_[id]];

            values2log_.push_back( pos_all[pos*3 + 0] );    //x
            values2log_.push_back( pos_all[pos*3 + 1] );    //y
            values2log_.push_back( pos_all[pos*3 + 2] );    //z
        }

        // Showing the orientation of movement of the robot
        for (double d : {0.0,1.0})
        {
            double x_dir = d*cos( uav_wished_yaw_map_ );
            double y_dir = d*sin( uav_wished_yaw_map_ );

            values2log_.push_back (x_dir);
            values2log_.push_back (y_dir);
        }

        // Showing the orientation requested from the system
        for (double d : {0.0,1.0})
        {
            double x_dir = d*cos( yaw_ref_ );
            double y_dir = d*sin( yaw_ref_ );

            values2log_.push_back (x_dir);
            values2log_.push_back (y_dir);
        }

        measurements info;
        // Saving the safety_region around the robot it is not necessary anymore

        // Saving the measurements around the robot
        for (unsigned id_phi = 0; id_phi < id_phi_max_; ++id_phi)
        {
            info = GetMeasurement( id_phi );

            double x_m = info.dist * cos( info.angle );
            double y_m = info.dist * sin( info.angle );

            values2log_.push_back (x_m);
            values2log_.push_back (y_m);
        }


        Log2Matlab(values2log_);
    }

}

/**
 * Prepares a logger for matlab
 */
void Swap_2_5d::Log2MatlabInit( const std::string file_path)
{
    // Loging first the parameters of the uav
    std::string log_var = file_path + "uav" + std::to_string(uav_id_) + "vars.txt";
    log2mat_.open(log_var.c_str(), std::ofstream::binary);

    values2log_.clear();

    values2log_.push_back(uav_safety_radius_);
    values2log_.push_back(bracking_distance_);
    values2log_.push_back(positioning_error_);

    Log2Matlab(values2log_);
    log2mat_.close();

    // Preparing to save the rest
    pos_all = new double[std::max(n_uavs_ * 3, 20)];
    std::string log_all = file_path + "uav" + std::to_string(uav_id_) + ".txt";
    log2mat_.open(log_all.c_str(), std::ofstream::binary);

    // Map UAV ids into log positions
    for(int id = 0; id < n_uavs_; id++)
    {
        log_pos_map_[uav_ids_[id]] = id;
    }
}

/**
 * Creates a log file that can be read in matlab
 */
void Swap_2_5d::Log2Matlab( std::vector<double>& values )
{
    if (log2mat_)
    {
        log2mat_ << std::setprecision(20);
        for (unsigned idx = 0; idx < values.size(); ++idx)
        {
            if (values[idx] < -10000 || values[idx] > 10000)
            {
                log2mat_ << " inf ";
            }
            else
            {
                log2mat_ << values[idx] << " ";
            }
        }
        log2mat_ << std::endl;
    }
}


