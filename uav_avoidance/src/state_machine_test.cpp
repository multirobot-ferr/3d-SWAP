/**
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
#include <tf2/utils.h>     // to convert quaternion to roll-pitch-yaw
#include <math.h>  // to use atan2

#include <uav_abstraction_layer/TakeOff.h>
#include <uav_abstraction_layer/SetVelocity.h>
#include <uav_abstraction_layer/SetPositionError.h>
#include <uav_abstraction_layer/GoToWaypoint.h>
#include <uav_abstraction_layer/Land.h>

/**
 * Main code of the state machine
 */
int main(int argc, char **argv) {
    // name remapping
    ros::init(argc, argv, "state_machine_test");


    StateMachine state_machine;

    ros::Rate sleeper(10);  // 10Hz

    while (ros::ok() && state_machine.Running())
    {
        // Performs all necessary actions
        state_machine.Loop();

        // Sleeping to save time
        //ros::Duration(0.1).sleep();
        sleeper.sleep();
    }

    // Forcing a landing out of the node
    state_machine.ForcedLand();


    return 0;
}

/**
 * Default constructor of the class
 */
StateMachine::StateMachine() {
    // Preparing private acquisition of parameters
    ROS_WARN("State Machine for testing awake! Disable it if you are not testing swap");

    // Sleeping some random time to not colapse the system due to a big wakeup
    // Waking a time between 1 and 2 seconds
    double sleep_time = 1.0 + (rand() % 10 + 1)/10.0;
    ROS_INFO("Waiting %f seconds before start", sleep_time);

    //pid_yaw_ = new grvc::utils::PidController("yaw", 0.4, 0.02, 0.0);

    ros::Duration( sleep_time).sleep();


    if (!pnh_.getParam("uav_id", uav_id_))
    {
        initialization_error = true;
        ROS_FATAL("State Machine: uav_id is not set. Closing the state machine system");
    }
    if (!pnh_.getParam("yaw_on", yaw_on_))
    {
        initialization_error = true;
        ROS_FATAL("State Machine: yaw_on is not set. Closing the state machine system");
    }
    if (!pnh_.getParam("v_max", v_ref_))
    {
        initialization_error = true;
        ROS_FATAL("State Machine: v_max is not set. Closing the state machine system");
    }
    if (!pnh_.getParam("game_frame", game_frame_))
    {
        initialization_error = true;
        ROS_FATAL("State Machine: game frame is not set. Closing the state machine system");
    }

    std::string goals_path;
    if (pnh_.getParam("goals_path", goals_path))
    {
        ROS_INFO("State Machine: Goal set:");
        way_points_.load(goals_path);
        
        
        if(game_frame_)
        {
            way_points_=gameToMap(way_points_, 2.1984);
        }

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

    // Specific namespace for UAL
    std::string ual_ns;
    if (!pnh_.getParam("ual_namespace", ual_ns))
        ual_ns = "";

    // Subscribing to the position of the UAV
    std::string uav_topic_name = "/" + ual_ns + "uav_" + std::to_string(uav_id_) + "/ual"+ pose_uav_topic.c_str();
    pos_uav_sub_ = nh_.subscribe(  uav_topic_name.c_str() , 1, &StateMachine::PoseReceived, this);
    // Preparing the necessary services

    std::string uav_service_name;
    uav_service_name = "/" + ual_ns + "uav_" +  std::to_string(uav_id_) + "/ual"+ takeoff_service;
    takeOff_srv_ = nh_.serviceClient<uav_abstraction_layer::TakeOff>(uav_service_name);

    uav_service_name = "/" + ual_ns + "uav_" + std::to_string(uav_id_) + "/ual" + speed_service;
    speed_srv_ = nh_.serviceClient<uav_abstraction_layer::SetVelocity>(uav_service_name);

    uav_service_name = "/" + ual_ns + "uav_" + std::to_string(uav_id_) + "/ual" + pos_err_service;
    pos_err_srv_ = nh_.serviceClient<uav_abstraction_layer::SetPositionError>(uav_service_name);

    uav_service_name = "/" + ual_ns + "uav_" + std::to_string(uav_id_) + "/ual" + way_point_service;
    way_point_srv_ = nh_.serviceClient<uav_abstraction_layer::GoToWaypoint>(uav_service_name);

    uav_service_name = "/" + ual_ns + "uav_" + std::to_string(uav_id_) + "/ual" + land_service;
    land_srv_ = nh_.serviceClient<uav_abstraction_layer::Land>(uav_service_name);


    pnh_.param<double>("z_distance", dist_between_uav_z_, 2.0);
    pnh_.param<double>("d_goal", d_goal_, 0.5);

    wait_for_start_ = nh_.advertiseService("Start", &StateMachine::StartServiceCb, this);

    // ###########  Communication with SWAP  ########## //
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
    //delete pnh_;
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
    for (auto i = 5; i > 0 && !landed_; --i)
    {
        ROS_INFO("Trying to land");
        
        uav_abstraction_layer::Land srv;
        srv.request.blocking = false;

        if (land_srv_.call(srv))
        {
            ROS_INFO("Landing");
            ros::Duration(10).sleep();
            landed_ = true;
        }
        else
        {
            ROS_ERROR("Failed to call service landing");
        }
        ros::Duration(3).sleep();
    }
}

/**
 * @brief Forces a land of the uav
 */
void StateMachine::ForcedLand()
{
    std::cout << "Forcing a landing out of the node" << std::endl;
    std::string command = "rosservice call /uav_" + std::to_string(uav_id_) + "/ual/land \"blocking: false\"";
    std::cout << command.c_str() << std::endl;
    system(command.c_str());
}

/**
 * @brief Returns true while the system still flying
 */
bool StateMachine::Running()
{
    return (!landed_);
}

/**
 * Callback for own pose estimation
 */
void StateMachine::PoseReceived(const geometry_msgs::PoseStamped::ConstPtr& uav_pose)
{
    tf2::Quaternion q3(uav_pose->pose.orientation.x,
                       uav_pose->pose.orientation.y,
                       uav_pose->pose.orientation.z,
                       uav_pose->pose.orientation.w);
    q3.normalize();     //Avoids a warning

    pose_received_ = true;
    uav_x_   = uav_pose->pose.position.x;
    uav_y_   = uav_pose->pose.position.y;
    uav_z_   = uav_pose->pose.position.z;
    uav_yaw_ = tf2::getYaw(q3);
}

bool StateMachine::StartServiceCb(std_srvs::SetBool::Request& allowed2move, std_srvs::SetBool::Response& ok)
{
    keep_moving_ = allowed2move.data;

    if (keep_moving_)
    {
        std::string msg = "Starting the experiment with uav" + std::to_string(uav_id_) + "!";
        ROS_INFO("%s", msg.c_str());
        ok.message = msg;
    }

    ok.success = true;
    return true;
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
            t = ros::Time::now();
            std::cout << "conflict!" << std::endl;
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

    //z is inluded in waypoints rigth now
    double xe = way_points_(wp_idx_, 0) - uav_x_;
    double ye = way_points_(wp_idx_, 1) - uav_y_;
    double ze = way_points_(wp_idx_, 2) - uav_z_;
    double dirx=xe/(sqrt(powf(xe, 2.0) + powf(ye, 2.0) + powf(ze, 2.0)));
    double diry=ye/(sqrt(powf(xe, 2.0) + powf(ye, 2.0) + powf(ze, 2.0)));
    double dirz=ze/(sqrt(powf(xe, 2.0) + powf(ye, 2.0) + powf(ze, 2.0)));
    
    // Information for SWAP (where the uav wants to go)
    wished_direction_uav_.x = xe;
    wished_direction_uav_.y = ye;
    wished_direction_uav_.z = ze;
    wished_mov_dir_pub_.publish(wished_direction_uav_);

    static bool last_confl_warning = true;
    if (confl_warning_ != last_confl_warning)
    {
        xv_pid_.resetI();
        yv_pid_.resetI();
        yawv_pid_.resetI();
        last_confl_warning = confl_warning_;
    }

    if (confl_warning_)
    {
        // Movement is not safe. Changing the reference
        xe = 0.8*v_ref_*avoid_mov_direction_uav_.x;
        ye = 0.8*v_ref_*avoid_mov_direction_uav_.y;
        ze = 0;
    }

    double yaw_desired=atan2(ye, xe);
    double yawe=ScaleAngle(yaw_desired)-ScaleAngle(uav_yaw_);


    double x_actuation = xv_pid_.control_signal(xe);
    double y_actuation = yv_pid_.control_signal(ye);
    double z_actuation = zv_pid_.control_signal(ze);
    double yaw_actuation= yawv_pid_.control_signal(yawe);



    if(yaw_on_)
    {
    
        ROS_ERROR("yaw_on active, system not available");

        // orientation is controlled
        PublishGRVCCmdVel(x_actuation, y_actuation, z_actuation, yaw_actuation);

    }
    else
    {

        PublishGRVCCmdVel(x_actuation, y_actuation, z_actuation);

    }



}
// ###########  #######################  ########### //

/**
 * Publish a position error on the controller of the uav
 */
void StateMachine::PublishGRVCPosErr(const double xe, const double ye, const double ze)
{
    uav_abstraction_layer::SetPositionError srv;
    srv.request.position_error.vector.x = xe;
    srv.request.position_error.vector.y = ye;
    srv.request.position_error.vector.z = ze;

    if (!pos_err_srv_.call(srv))
    {
        ROS_ERROR("Failed to call service SetPositionError");
    }
}

/**
 * Publishes a speed on the grvc controler
 */
void StateMachine::PublishGRVCCmdVel(const double vx, const double vy,
                                     const double vz, const double yaw_rate)
{
    uav_abstraction_layer::SetVelocity srv;
    srv.request.velocity.twist.linear.x = vx;
    srv.request.velocity.twist.linear.y = vy;
    srv.request.velocity.twist.linear.z = vz;
    srv.request.velocity.twist.angular.z = yaw_rate;
    if (!speed_srv_.call(srv))
    {
        ROS_ERROR("Failed to call service SetVelocity");
    }
}

/**
 * Publishes a goal on the grvc controller
 */
void StateMachine::PublishGRVCgoal(const double x, const double y, const double z, const double yaw)
{
    uav_abstraction_layer::GoToWaypoint srv;
    srv.request.blocking = false;
    srv.request.waypoint.pose.position.x = x;
    srv.request.waypoint.pose.position.y = y;
    srv.request.waypoint.pose.position.z = z;


    tf2::Quaternion q3;
    q3.setRPY(0.0,0.0,yaw); 

    srv.request.waypoint.pose.orientation = tf2::toMsg(q3);


    if (!way_point_srv_.call(srv))
    {
        ROS_ERROR("Failed to call service GoToWaypoint");
    }
}

/**
 * Takes off the quadrotor
 */
void StateMachine::TakeOff()
{
    ROS_INFO("State Machine: TakingOff the UAV");
    // Leaving some extra time for the UAL to wake up

    while ( !takeOff_srv_.exists() )
    {
        ROS_INFO("Waiting for connection");
        ros::Duration(1).sleep();
    }

    z_ref_ = std::max(1.0, way_points_(0, 2));

    uav_abstraction_layer::TakeOff srv;
    srv.request.blocking = true;
    srv.request.height = z_ref_;

    if (takeOff_srv_.call(srv))
    {

        ros::Duration(3).sleep();

        // Waiting for the UAV to really take off
       /* while ( abs(uav_z_ - z_ref_) > 0.5 )
        {
            ros::Duration(3).sleep();
            ROS_INFO("take off done");
        } */
        ROS_WARN("UAV_%d ready (hovering at %.2f)", uav_id_, z_ref_);   
    }
    else
    {
        ROS_ERROR("Failed to call service TakingOff");
    }
}

/**
 * If the robot is close to a waypoint, it actualizes to the next waypoint
 */
void StateMachine::UpdateWayPoints()
{
    static bool msg_shown = false;

    double x_diff = way_points_(wp_idx_, 0) - uav_x_;
    double y_diff = way_points_(wp_idx_, 1) - uav_y_;
    double z_diff = way_points_(wp_idx_, 2) - uav_z_;

    double dist = sqrt(powf(x_diff, 2.0) + powf(y_diff, 2.0) + powf(z_diff, 2.0));

    if (dist < d_goal_) {
        // Stopping the uav a little bit there
        if (!msg_shown)
        {
            ROS_INFO("UAV_%d: Goal achieved (%.2fm to the goal)", uav_id_, dist);
            msg_shown = true;
        }

        if (wp_idx_ == way_points_.n_rows - 1)
        {
            experiment_done = true;
            Land();
        }

        if (keep_moving_ && !experiment_done)
        {
            msg_shown = false;
            
            // Remaining two seconds on the position
            ros::Rate sleeper(10);  // 10Hz
            ros::Time start = ros::Time::now();
            ros::Duration duration;
            do{
                duration = ros::Time::now() - start;
                PublishPosErr();
                sleeper.sleep();
            }while (duration.toSec() < 5.0 && ros::ok());

            ++wp_idx_;
            if (wp_idx_ >= way_points_.n_rows) {
                // experiment_done = true;
                // wp_idx_ = way_points_.n_rows;
                // Land();
            }
            else
            {
                ROS_INFO("UAV_%d: New goal set (%d/%d): %.2f, %.2f, %.2f", 
                        uav_id_, wp_idx_+1, int(way_points_.n_rows)+1,
                        way_points_(wp_idx_, 0), way_points_(wp_idx_, 1), way_points_(wp_idx_, 2));
                xv_pid_.reset();
                yv_pid_.reset();
                zv_pid_.reset();
            }
        }
    }
    else
    {
        // ROS_INFO("UAV_%d: Distance to goal %.2f", uav_id_, dist);
    }
}

/**
 * Utility function. Returns an angle between (-pi and pi)
 */
double StateMachine::ScaleAngle(double angle)
{
    while (angle < -M_PI)
    {
        angle += 2.0*M_PI;
    }
    while (angle > +M_PI)
    {
        angle -= 2.0*M_PI;
    }
    return angle;
}

/**
 * Utility function. If game_frame_ is true it returns waypoints in /map
 */

arma::mat StateMachine::gameToMap(arma::mat wp_game, double yaw_rot)
{

    arma::mat wp_map=wp_game;
        for (int row = 0; row < way_points_.n_rows; ++row)
            {
            wp_map(row,0)=wp_game(row,0)*cos(yaw_rot)-wp_game(row,1)*sin(yaw_rot);
            wp_map(row,1)=wp_game(row,0)*sin(yaw_rot)+wp_game(row,1)*cos(yaw_rot);
            
            }
            
    return wp_map;

}
