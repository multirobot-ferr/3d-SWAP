#ifndef SWAP_2_5D_H
#define SWAP_2_5D_H

#include "ros/ros.h"
#include "./core/swap.h"                    // base class
//
//// messages need
//#include "sensor_msgs/LaserScan.h"      // laser ranger
//#include <pcl_ros/point_cloud.h>        // for debug visualisation
//#include "geometry_msgs/PoseStamped.h"  // receives goals
//#include "tf/transform_datatypes.h"     // to convert quaternion to roll-pitch-yaw
//#include "geometry_msgs/PoseWithCovarianceStamped.h"    // receives positions
//#include "geometry_msgs/Twist.h"        // publishes movements
//
//namespace PLCDebug
//{
//    typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
//
//    // Color definitions for the PLC
//    struct PLC_COLOR {unsigned R; unsigned G; unsigned B;};
//    const  PLC_COLOR LIGHT_BLUE {  0, 255, 255};
//    const  PLC_COLOR DARK_BLUE  {  0,   0, 255};
//    const  PLC_COLOR GREEN      {  0, 255,   0};
//    const  PLC_COLOR ORANGE     {255, 159,   0};
//    const  PLC_COLOR RED        {255,   0,   0};
//    const  PLC_COLOR BLACK      {  0,   0,   0};
//    const  PLC_COLOR YELLOW     {255, 255,   0};
//
//}   // namespace PLCDebug
//
//
class Swap_2_5d: public avoid::Swap
{
    public:
//
//        /**
//         * @brief Default constructor of the class
//         *
//         * Request to ROS the necessary variables from the parameters and connects the necessary
//         * publishers and subscribers.
//         */
//        SwapROS();
//
//        /**
//         * @brief Performs all necessary publications
//         */
//        void Publish();
//
    private:
        ros::NodeHandle _nh;            //< ROS Node handler
//
//        // Subscribers
//        ros::Subscriber laser_sub_;     //!< Receives laser scans and fills the polar obstacle diagram
//        ros::Subscriber goal_sub_;      //!< Receives single goal poses
//        ros::Subscriber pose_sub_;      //!< Receives pose (position and orientation) estimates
//
//        // Publishers
//        ros::Publisher cmd_vel_pub_;    //!< Publishes new command velocity after collision avoidance calculations
//
//        // Variables required for the debuging process
//        bool debug_;                            //!< Activates debug output of the system
//        ros::Publisher debug_pub_;              //!< If debug: Publishes a point cloud for debug visualisation purposes
//        std::string debug_frame_id_;            //!< Stores the frame_id where for the debuging point cloud
//
//        // Ranger related variables
//        bool laser_asume_dynamic_ = true;       //!< Assumes laser measurements dynamics or not
//
//        // Localization and navigation variables
//        bool    pose_received_ = false;         //!< Boolean to track the first position detection
//        double  robot_x_, robot_y_, robot_yaw_; //!< Current position of the robot with respect to the map
//
//        bool    goal_map_received_ = false;     //!< The last coal depends on the frame of the map
//        bool    goal_robot_received_ = false;   //!< The last coal depends on the frame of the robot
//        double  goal_x_, goal_y_, goal_yaw_;    //!< Current position of the goal with respect to the map
//
//        double  v_ref_, yaw_ref_;               //!< References for the robot
//
//
//        /* Callbacks for ROS */
//
//        /**
//         * @brief Callback for a laser ranger message
//         *
//         * @param laser_scan_msg A laser scann meassage.
//         */
//        void LaserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan_msg);
//
//        /**
//         * @brief Callback for goals.
//         *
//         * @param msg Goal message, typically comming from topic "move_base_simple/goal"
//         */
//        void GoalReceived(const geometry_msgs::PoseStamped& goal_msg);
//
//        /**
//         * @brief Callback for own pose estimates
//         *
//         * @param msg Position message, typically comming from topic "amcl_pose"
//         */
//        void PoseReceived(const geometry_msgs::PoseWithCovarianceStamped& pose_msg);
//
//        /* Internal publishers */
//
//        /**
//         * @brief Computes the necessary control actions for the robot and publishes them
//         */
//        void PublishCmdVel();
//
//        /**
//         * @brief Publish a debuging PointCloud
//         *
//         * Publishes a PointCloud that helps the user to understand what is happening
//         */
//        void PublishDebugPCL();
//
//        /* Internal functions */
//        /**
//         *  @brief Converts from different frame id to submit the necessary goal position and orientation
//         */
//        void CommandGoals();
//
//        /**
//         * @brief Utility function. Set the color of the specified point cloud
//         *
//         * \param[in]  color Specified color
//         * \param[out] point Point cloud variable
//         */
//        void SetPCLcolor(pcl::PointXYZRGB& point, PLCDebug::PLC_COLOR color);
//
}; // class SwapRos



#endif // SWAP_2_5D_H
