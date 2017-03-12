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
 * @file state_machine_test.cpp
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

#include <uav_avoidance/state_machine_test.h>

/**
 * Main code of the state machine
 */
int main(int argc, char **argv) {
    // name remapping
    ros::init(argc, argv, "state_machine_test");

    StateMachine state_machine( argc, argv );

    while (ros::ok())
    {
        ros::spinOnce();

        // Performs all necessary actions
        state_machine.Loop();

        // Sleeping to save time
        ros::Duration(0.1).sleep();
    }

    state_machine.Land();

    return 0;
}

/**
 * Default constructor of the class
 */
StateMachine::StateMachine(int argc, char** argv): argc_(argc), argv_(argv) {
    // Preparing private acquisition of parameters
    ROS_WARN("State Machine for testing awake! Disable it if you are not testing swap");
    pnh_ = new ros::NodeHandle("~");

    // Sleeping some random time to not colapse the system due to a big wakeup
    // Waking a time between 1 and 2 seconds
    double sleep_time = 1.0 + (rand() % 10 + 1)/10.0;
    ROS_INFO("Waiting %f seconds before start", sleep_time);
    ros::Duration( sleep_time).sleep();


    if (!pnh_->getParam("uav_id", uav_id_))
    {
        initialization_error = true;
        ROS_FATAL("State Machine: uav_id is not set. Closing the state machine system");
    }

    std::string goals_path;
    if (pnh_->getParam("goals_path", goals_path))
    {
        ROS_INFO("State Machine: Goal set:");
        way_points_.load(goals_path);
        for (int row = 0; row < way_points_.n_rows; ++row)
        {
            ROS_INFO("Goal id=%d (%.2f, %.2f, %.2f)", row,
                     way_points_(row, 0), way_points_(row, 1), way_points_(row, 2));
        }
    }


    #ifdef UAV_NOT_IN_ZERO_ZERO
    for (auto row = 0; row < way_points_.n_rows; ++row)
    {
        for (auto col = 0; col < way_points_.n_cols; ++col)
        {
            way_points_(row, col) -= UAV_ZZ(uav_id_ - 1, col);
        }
    }
    #endif

    // Subscribing to the position of the UAV
    std::string uav_topic_name = "/mbzirc_" + std::to_string(uav_id_) + pose_uav_topic.c_str();
    pos_uav_sub_ = nh_.subscribe(uav_topic_name.c_str(), 1, &StateMachine::PoseReceived, this);

    // Preparing the necessary services
    grvc::utils::ArgumentParser args(argc_, argv_);

    std::string uav_service_name;
    uav_service_name = "/mbzirc_" + std::to_string(uav_id_) + takeoff_service;
    takeOff_srv_ = new grvc::hal::Server::TakeOffService::Client(uav_service_name, args);

    uav_service_name = "/mbzirc_" + std::to_string(uav_id_) + speed_service;
    speed_srv_ = new grvc::hal::Server::VelocityService::Client( uav_service_name, args);

    uav_service_name = "/mbzirc_" + std::to_string(uav_id_) + pos_err_service;
    pos_err_srv_ = new grvc::hal::Server::PositionErrorService::Client( uav_service_name, args);

    uav_service_name = "/mbzirc_" + std::to_string(uav_id_) + way_point_service;
    way_point_srv_ = new grvc::hal::Server::WaypointService::Client(uav_service_name, args);

    uav_service_name = "/mbzirc_" + std::to_string(uav_id_) + land_service;
    land_srv_  = new grvc::hal::Server::LandService::Client( uav_service_name, args);


    pnh_->param<double>("z_distance", dist_between_uav_z_, 2.0);
    pnh_->param<double>("d_goal", d_goal_, 0.5);

    // ###########  Communication with SWAP  ########### //
    confl_warning_sub_  = nh_.subscribe("collision_warning", 1, &StateMachine::WarningCallback, this);
    wished_mov_dir_pub_ = nh_.advertise<geometry_msgs::Vector3>("wished_movement_direction",1 , true);  // the final true is required
    avoid_mov_dir_sub_  = nh_.subscribe("avoid_movement_direction", 1, &StateMachine::AvoidMovementCallback, this);
    // ###########  #######################  ########### //

    TakeOff();

    if (!initialization_error)
    {
        ROS_INFO("State Machine: Init complete");
    }
}

/**
 * Destructor to release the memory
 */
StateMachine::~StateMachine()
{
    // Releasing the memory
    delete pnh_;
    delete takeOff_srv_;
    delete speed_srv_;
    delete way_point_srv_;
    delete land_srv_;
}

/**
 * Performs all necessary actions on the loop
 */
void StateMachine::Loop()
{
    // Moves to the necessary position
    PublishPosErr();

    // If necessary, goes to the next waypoint of the list
    UpdateWayPoints();
}

/**
 * Lands the uav
 */
void StateMachine::Land()
{
    //TODO For some reason this thing does not work.
    // Landing the quadcopter (if possible)
    ROS_INFO("Trying to land");
    if (land_srv_)
    {
        ROS_INFO("Landing");
        land_srv_->send(ts_);
        ros::Duration(10).sleep();
    }
}

/**
 * Callback for own pose estimation
 */
void StateMachine::PoseReceived(const std_msgs::String::ConstPtr& uav_pose)
{
    std::stringstream msg;
    msg << uav_pose->data;
    grvc::hal::Pose pose;
    msg >> pose;

    tf::Quaternion q3( pose.orientation[0],
                       pose.orientation[1],
                       pose.orientation[2],
                       pose.orientation[3]);
    q3.normalize();     //Avoids a warning

    pose_received_ = true;
    uav_x_   = pose.position[0];
    uav_y_   = pose.position[1];
    uav_z_   = pose.position[2];
    uav_yaw_ = tf::getYaw(q3);
}

// ###########  Communication with SWAP  ########### //
/**
 * Callback from SWAP that informs to the state machine of a conflict
 * NOTE: You can copy it and paste in your code as it is.
 */
void StateMachine::WarningCallback(const std_msgs::Bool::ConstPtr& collision_warning)
{
    static bool transition = false;
    confl_warning_ = collision_warning->data;

    if (transition != confl_warning_)
    {
        transition = confl_warning_;
        if (confl_warning_)
        {
            ROS_WARN("Conflict warning");
        }
        else
        {
            ROS_INFO("Back to normal control");
        }
    }
}
// ###########  #######################  ########### //

// ###########  Communication with SWAP  ########### //
/**
 * Informs to the state machine of the direction to take in case of a conflict
 * NOTE: You can copy it and paste in your code as it is.
 */
void StateMachine::AvoidMovementCallback(const geometry_msgs::Vector3::ConstPtr& avoidance_direction)
{
    // Receives an avoidance direction (when is necessary to take one)
    avoid_mov_direction_uav_.x = avoidance_direction->x;
    avoid_mov_direction_uav_.y = avoidance_direction->y;
    avoid_mov_direction_uav_.z = avoidance_direction->z; // Ignorable (always 0)
}
// ###########  #######################  ########### //

// ###########  Communication with SWAP  ########### //
/**
 * Computes the necessary control actions for the robot and publishes them
 * NOTE: You CANNOT copy direclty this code. You should addapt this code to
 * your state machine.
 */
void StateMachine::PublishPosErr()
{
    // Getting fresh information on the position and warnings
    ros::spinOnce();

    double xe = way_points_(wp_idx_, 0) - uav_x_;
    double ye = way_points_(wp_idx_, 1) - uav_y_;
    double ze = z_ref_ - uav_z_;

    // Information for SWAP (where the uav wants to go)
    wished_direction_uav_.x = xe;
    wished_direction_uav_.y = ye;
    wished_direction_uav_.z = ze;
    wished_mov_dir_pub_.publish(wished_direction_uav_);

    // Information for the UAV
    if (!confl_warning_)
    {
        // Move is safe
        PublishGRVCPosErr(xe, ye, ze);
    }
    else
    {
        // Move is not safe
        // Speed controller seems to work better with SWAP
        PublishGRVCPosErr( avoid_mov_direction_uav_.x, avoid_mov_direction_uav_.y, ze);
        //PublishGRVCCmdVel( avoid_mov_direction_uav_.x, avoid_mov_direction_uav_.y, ze, 0.0);
    }
}
// ###########  #######################  ########### //

/**
 * Publish a position error on the controller of the uav
 */
void StateMachine::PublishGRVCPosErr(const double xe, const double ye, const double ze)
{
    if (pos_err_srv_)
    {
        //ROS_INFO("Sending position error %.2f,%.2f,%.2f", xe, ye, ze);
        grvc::hal::Vec3 pos_error(xe, ye, ze);
        pos_err_srv_->send(pos_error, ts_);
    }
}

/**
 * Publishes a speed on the grvc controler
 */
void StateMachine::PublishGRVCCmdVel(const double vx, const double vy,
                                     const double vz, const double yaw_rate)
{
    if (speed_srv_)
    {
        // Swap has control of the speed too
        grvc::hal::Velocity velocity;
        velocity.linear[0] = vy;
        velocity.linear[1] = vy;
        velocity.linear[2] = vz;
        velocity.yaw_rate  = yaw_rate;

        speed_srv_->send( velocity, ts_);
    }
}

/**
 * Publishes a goal on the grvc controler
 */
void StateMachine::PublishGRVCgoal(const double x, const double y, const double z, const double yaw)
{
    if (way_point_srv_)
    {
        // Swap has control of the movements
        grvc::hal::Waypoint way_point = {{ x, y, z}, yaw};
        way_point_srv_->send(way_point, ts_);   // Blocking!
    }
}

/**
 * Takes off the quadrotor
 */
void StateMachine::TakeOff()
{
    // Getting fresh information on the position
    ros::spinOnce();

    if (takeOff_srv_ )
    {
        ROS_INFO("State Machine: TakingOff the UAV");
        // Leaving some extra time for the hal to wake up
        ros::Duration(1).sleep();

        while ( !takeOff_srv_->isConnected() )
        {
            ROS_INFO("Waiting for connection");
            ros::Duration(1).sleep();
        }

        z_ref_ = std::max(dist_between_uav_z_ * uav_id_, 1.0);
        takeOff_srv_->send(z_ref_, ts_);   // Blocking!


        // Waiting for the UAV to really take off
        while ( abs(uav_z_ - z_ref_) > 0.5 )
        {
            ros::Duration(1).sleep();
        }
        ROS_WARN("UAV_%d ready (hovering at %.2f)", uav_id_, z_ref_);

    }
    else
    {
        ROS_FATAL("Takeoff system not connected");
    }
}

/**
 * If the robot is close to a waypoint, it actualizes to the next waypoint
 */
void StateMachine::UpdateWayPoints()
{
    // Getting fresh information
    ros::spinOnce();

    double x_diff = way_points_(wp_idx_, 0) - uav_x_;
    double y_diff = way_points_(wp_idx_, 1) - uav_y_;

    double dist = sqrt(powf(x_diff, 2.0) + powf(y_diff, 2.0));

    if (dist < d_goal_) {
        // Stopping the uav a little bit there
        ROS_INFO("UAV_%d: Goal achieved (%.2fm to the goal)", uav_id_, dist);
        for (auto i = 0; i< 2; ++i)
        {
            //PublishGRVCPosErr(0.0, 0.0, 0.0);
            PublishGRVCgoal(way_points_(wp_idx_, 0), way_points_(wp_idx_, 1),
                            z_ref_, way_points_(wp_idx_, 2));   //Blocking
            ros::Duration(1).sleep();
        }

        ++wp_idx_;
        if (wp_idx_ >= way_points_.n_rows) {
            wp_idx_ = 0;
        }
        ROS_INFO("UAV_%d: New goal set: %.2f,%.2f", uav_id_, way_points_(wp_idx_, 0), way_points_(wp_idx_, 1));
    }
    else
    {
        //ROS_INFO("UAV_%d: Distance to goal %.2f", uav_id_, dist);
    }
}