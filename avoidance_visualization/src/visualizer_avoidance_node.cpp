#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <tf2/utils.h>
#include <string>
#include <sstream>
#include <vector>
#include <map>
#include <fstream>
#include <iostream>
#include <math.h>
#include <ros/ros.h>

using namespace std;


/** This class subscribes to avoidance_swap data to publish visual marker
*/

class Visualizer
{

public:
    Visualizer();
    ~Visualizer();

    void publishMarkers();
    

protected:
    
    ///Callbacks

    void uavPoseReceived1(const geometry_msgs::PoseStamped::ConstPtr& uav_pose);
    void uavPoseReceived2(const geometry_msgs::PoseStamped::ConstPtr& uav_pose);
    void uavPoseReceived3(const geometry_msgs::PoseStamped::ConstPtr& uav_pose);

    void AvoidMovementCallback1(const geometry_msgs::Vector3::ConstPtr& avoidance_direction);
    void AvoidMovementCallback2(const geometry_msgs::Vector3::ConstPtr& avoidance_direction);
    void AvoidMovementCallback3(const geometry_msgs::Vector3::ConstPtr& avoidance_direction);

    
    void GoalDirectionCallback1(const geometry_msgs::Vector3::ConstPtr& goal_direction);
    void GoalDirectionCallback2(const geometry_msgs::Vector3::ConstPtr& goal_direction);
    void GoalDirectionCallback3(const geometry_msgs::Vector3::ConstPtr& goal_direction);
    
    
    
    

    ///Node handlers
    ros::NodeHandle* nh_;
    ros::NodeHandle* pnh_;

    /// Subscribers

    vector<ros::Subscriber *> uav_subs_;
    vector<ros::Subscriber *> direction_subs_;
    vector<ros::Subscriber *> goal_direction_subs_;
    
    

    /// Publishers
    ros::Publisher scenario_pub_;          // Publisher for scenario marker
    ros::Publisher cylinder_pub_;          // Publisher for cylinder marker
    ros::Publisher uavs_pub_;              // Publisher for uav pose
    ros::Publisher arrow_pub_;             // Publisher for avoidance direction marker
    ros::Publisher goal_arrow_pub_;        // Publisher for goal direction marker
    ros::Publisher uav_cylinder_pub_;    // Publisher for safety cylinder

    /// Last data received
    map<int, geometry_msgs::PoseStamped> uavs_poses_;
    map<int, geometry_msgs::Vector3> uav_direction_;
    map<int, geometry_msgs::Vector3> goal_direction_;

    /// Number of UAVs
    int n_uavs_;
    double uav_safety_radius_ = -1.0;               //!< Safety radius of each uav
    double dz_min_;	    		   		            //!< max z-distance to swap. It is a parameter
    double bracking_distance_;
    double dz_range_;
    double positioning_error_;
    double gamma_offset_;
};

/** \brief Constructor
*/
Visualizer::Visualizer()
{

    nh_ = new ros::NodeHandle();
    pnh_ = new ros::NodeHandle("~");
    

    // getting swap parameters


    pnh_->param<int>("n_uavs", n_uavs_, 3);

    if (!pnh_->getParam("safety_radius", uav_safety_radius_))
    {
        ROS_FATAL("VISUALIZER: safety radius is not set. Closing the visualizer system");
    }

    if (!pnh_->getParam("braking_distance", bracking_distance_))
    {
        ROS_FATAL("VISUALIZER: breaking distance is not set. Closing the visualizer system");
    }
    if (!pnh_->getParam("dz_min", dz_min_))
    {
        ROS_FATAL("VISUALIZER: cylinder height is not set. Closing the visualizer system");
    }
    if (!pnh_->getParam("dz_range", dz_range_))
    {
        ROS_FATAL("VISUALIZER: cylinder height is not set. Closing the visualizer system");
    }

    if (!pnh_->getParam("positioning_error", positioning_error_))
    {
        ROS_FATAL("VISUALIZER: positioning error is not set. Closing the visualizer system");
    }

    if (!pnh_->getParam("gamma_offset", gamma_offset_))
    {
        ROS_FATAL("VISUALIZER: gamma offset is not set. Closing the visualizer system");
    }


    // Subscriptions/publications
    for(int i=0; i<n_uavs_; i++)
    {
        string uav_topic_name = "uav_" + to_string(i+1) + "/ual/pose";
        string uav_direction_topic_name =  "uav_" + to_string(i+1) + "/avoid_movement_direction";
        string goal_direction_topic_name= "uav_" + to_string(i+1) + "/wished_movement_direction";
        
        if(i==0){
            ROS_INFO("subscribiendo a pos 1");
            ros::Subscriber* uav_sub= new ros::Subscriber();
            *uav_sub = nh_->subscribe<geometry_msgs::PoseStamped>(uav_topic_name.c_str(), 1, &Visualizer::uavPoseReceived1, this);
            uav_subs_.push_back(uav_sub);

            ros::Subscriber* direction_sub= new ros::Subscriber();
            *direction_sub = nh_->subscribe<geometry_msgs::Vector3>(uav_direction_topic_name.c_str(), 1, &Visualizer::AvoidMovementCallback1, this);
            direction_subs_.push_back(direction_sub);

            ros::Subscriber* goal_direction_sub= new ros::Subscriber();
            *goal_direction_sub = nh_->subscribe<geometry_msgs::Vector3>(goal_direction_topic_name.c_str(), 1, &Visualizer::GoalDirectionCallback1, this);
            goal_direction_subs_.push_back(goal_direction_sub);



        }
        else if(i==1)
        {
            ROS_INFO("subscribiendo a pos 2");
            ros::Subscriber* uav_sub= new ros::Subscriber();
            *uav_sub = nh_->subscribe<geometry_msgs::PoseStamped>(uav_topic_name.c_str(), 1, &Visualizer::uavPoseReceived2, this);
            uav_subs_.push_back(uav_sub);


            ros::Subscriber* direction_sub= new ros::Subscriber();
            *direction_sub = nh_->subscribe<geometry_msgs::Vector3>(uav_direction_topic_name.c_str(), 1, &Visualizer::AvoidMovementCallback2, this);
            direction_subs_.push_back(direction_sub);

            ros::Subscriber* goal_direction_sub= new ros::Subscriber();
            *goal_direction_sub = nh_->subscribe<geometry_msgs::Vector3>(goal_direction_topic_name.c_str(), 1, &Visualizer::GoalDirectionCallback2, this);
            goal_direction_subs_.push_back(goal_direction_sub);
        }
        else if(i==2)
        {
            ROS_INFO("subscribiendo a pos 3");
            
            ros::Subscriber* uav_sub= new ros::Subscriber();
            *uav_sub = nh_->subscribe<geometry_msgs::PoseStamped>(uav_topic_name.c_str(), 1, &Visualizer::uavPoseReceived3, this);
            uav_subs_.push_back(uav_sub);

            ros::Subscriber* direction_sub= new ros::Subscriber();
            *direction_sub = nh_->subscribe<geometry_msgs::Vector3>(uav_direction_topic_name.c_str(), 1, &Visualizer::AvoidMovementCallback3, this);
            direction_subs_.push_back(direction_sub);

            ros::Subscriber* goal_direction_sub= new ros::Subscriber();
            *goal_direction_sub = nh_->subscribe<geometry_msgs::Vector3>(goal_direction_topic_name.c_str(), 1, &Visualizer::GoalDirectionCallback3, this);
            goal_direction_subs_.push_back(goal_direction_sub);
        }
    
        
    }


    scenario_pub_ = nh_->advertise<visualization_msgs::Marker>("avoidance_markers/scenario", 0);
    uavs_pub_ = nh_->advertise<visualization_msgs::MarkerArray>("avoidance_markers/uavs", 0);
    cylinder_pub_= nh_->advertise<visualization_msgs::MarkerArray>("avoidance_markers/cylinders",0);
    arrow_pub_= nh_ ->advertise<visualization_msgs::MarkerArray>("avoidance_markers/arrow",0);
    goal_arrow_pub_ = nh_ -> advertise<visualization_msgs::MarkerArray>("avoidance_markers/arrow_goal",0);
    uav_cylinder_pub_ = nh_ -> advertise<visualization_msgs::MarkerArray>("avoidance_markers/safety_cylinder", 0);

}

/** \brief Destructor
*/
Visualizer::~Visualizer()
{
    delete nh_;
    delete pnh_;

    for(int i=0; i < n_uavs_; i++)
    {
        delete uav_subs_[i];
        delete direction_subs_[i];
        delete goal_direction_subs_[i];
    }
    
    uavs_poses_.clear();
    uav_subs_.clear();
    uav_direction_.clear();
    direction_subs_.clear();
    goal_direction_.clear();
}


/** \brief Callback to receive UAVs poses
*/
void Visualizer::uavPoseReceived1(const geometry_msgs::PoseStamped::ConstPtr& uav_pose)
{
    int uav_id=1; // TODO get uav_id
	//int uav_id = stoi(pose.id);
	if(0 < uav_id && uav_id <= n_uavs_)
        uavs_poses_[uav_id] = *uav_pose;
}

void Visualizer::uavPoseReceived2(const geometry_msgs::PoseStamped::ConstPtr& uav_pose)
{
    int uav_id= 2; // TODO get uav_id
	//int uav_id = stoi(pose.id);
	if(0 < uav_id && uav_id <= n_uavs_)
        uavs_poses_[uav_id] = *uav_pose;
}

void Visualizer::uavPoseReceived3(const geometry_msgs::PoseStamped::ConstPtr& uav_pose)
{
    int uav_id= 3; // TODO get uav_id
	//int uav_id = stoi(pose.id);
	if(0 < uav_id && uav_id <= n_uavs_)
        uavs_poses_[uav_id] = *uav_pose;
}

/** \brief Callback for avoid direction
*/

void Visualizer::AvoidMovementCallback1(const geometry_msgs::Vector3::ConstPtr& avoidance_direction)
{
    // Receives an avoidance direction (when is necessary to take one)
    int uav_id=1;
    uav_direction_[uav_id] = *avoidance_direction;
}

void Visualizer::AvoidMovementCallback2(const geometry_msgs::Vector3::ConstPtr& avoidance_direction)
{
    // Receives an avoidance direction (when is necessary to take one)
    int uav_id=2;
    uav_direction_[uav_id] = *avoidance_direction;
}

void Visualizer::AvoidMovementCallback3(const geometry_msgs::Vector3::ConstPtr& avoidance_direction)
{
    // Receives an avoidance direction (when is necessary to take one)
    int uav_id=3;
    uav_direction_[uav_id] = *avoidance_direction;
}

/** \Callback for goal direction
*/

void Visualizer::GoalDirectionCallback1(const geometry_msgs::Vector3::ConstPtr& goal_direction)
{
    int uav_id=1;
    goal_direction_[uav_id] = *goal_direction;
}

void Visualizer::GoalDirectionCallback2(const geometry_msgs::Vector3::ConstPtr& goal_direction)
{
    int uav_id=2;
    goal_direction_[uav_id] = *goal_direction;
}

void Visualizer::GoalDirectionCallback3(const geometry_msgs::Vector3::ConstPtr& goal_direction)
{
    int uav_id=3;
    goal_direction_[uav_id] = *goal_direction;
}


/** Publish markers
*/
void Visualizer::publishMarkers()
{

    // Publish the scenario
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();
    marker.ns = "scenario";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://mbzirc_visualization/meshes/stage.dae";
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = -0.99893;
    marker.pose.orientation.w = 0.046306;

    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;

    marker.mesh_use_embedded_materials = true;
    marker.color.a = 0.5;
    marker.lifetime = ros::Duration();

    scenario_pub_.publish(marker);

    // Publish UAVs and cylinder
    visualization_msgs::MarkerArray uav_markers;
    visualization_msgs::MarkerArray cylinder_markers;
    visualization_msgs::MarkerArray uav_cylinder_markers;
    visualization_msgs::MarkerArray arrow_markers;
    visualization_msgs::MarkerArray arrow_goal_markers;

    for (int uav_id = 1; uav_id <= n_uavs_; uav_id++)
    {
        //uav_marker
        // If not empty (never received)
        if(uavs_poses_.find(uav_id) != uavs_poses_.end())
        {

            visualization_msgs::Marker marker;
            marker.header.frame_id = "/map";
            marker.header.stamp = ros::Time();
            marker.id = uav_id;
            marker.ns = "uavs";
            marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            marker.mesh_resource = "package://robots_description/models/mbzirc/meshes/multirotor.dae";
            marker.color.a = 1;    
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose = uavs_poses_[uav_id].pose;

            marker.scale.x = 0.001;
            marker.scale.y = 0.001;
            marker.scale.z = 0.001;
            marker.mesh_use_embedded_materials = true;

            switch(uav_id)
            {
                case 1:
                // orange
                marker.color.r = 1.0;
                marker.color.g = 0.647;
                marker.color.b = 0.0;
                break;
                case 2:
                // ingigo
                marker.color.r = 0.294;
                marker.color.g = 0.0;
                marker.color.b = 0.510; 
                break;
                case 3:
                // zinc yellow
                marker.color.r = 0.945;
                marker.color.g = 0.812;
                marker.color.b = 0.267;
                break;
            }

            uav_markers.markers.push_back(marker);

            marker.ns = "uavs_state";
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.text = std::to_string(uav_id) + " ";
            marker.pose.position.z += 2.0;
            
            marker.scale.x = 1;
            marker.scale.y = 1;
            marker.scale.z = 1;
            
            // Publish uav cylinder

            visualization_msgs::Marker uav_cylinder;
            uav_cylinder.header.frame_id = "/map";
            uav_cylinder.header.stamp = ros::Time();
            uav_cylinder.id = uav_id;
            uav_cylinder.ns = "uavs";
            uav_cylinder.type = visualization_msgs::Marker::CYLINDER;
            uav_cylinder.color.a = 0.1;   

            switch(uav_id)
            {
                case 1:
                
                uav_cylinder.color.r = 0.7;
                uav_cylinder.color.g = 0;
                uav_cylinder.color.b = 0;
                break;
                case 2:

                uav_cylinder.color.r = 0;
                uav_cylinder.color.g = 0.7;
                uav_cylinder.color.b = 0; 
                break;
                case 3:
                uav_cylinder.color.r = 0.7;
                uav_cylinder.color.g = 0.7;
                uav_cylinder.color.b = 0;
                break;
            }

            uav_cylinder.action = visualization_msgs::Marker::ADD;

            uav_cylinder.pose = uavs_poses_[uav_id].pose;
            uav_cylinder.scale.x = 2*uav_safety_radius_;   //Diameter
            uav_cylinder.scale.y = 2*uav_safety_radius_;   // if x e y are different you get an elipse instead o a circle
            uav_cylinder.scale.z = dz_min_;   // height
            uav_cylinder.mesh_use_embedded_materials = true;

            uav_cylinder_markers.markers.push_back(uav_cylinder);

            // Publish cylinder

            visualization_msgs::Marker cylinder;
            cylinder.header.frame_id = "/map";
            cylinder.header.stamp = ros::Time();
            cylinder.id = uav_id;
            cylinder.ns = "uavs";
            cylinder.type = visualization_msgs::Marker::CYLINDER;
            cylinder.color.a = 0.1;   

            switch(uav_id)
            {
                case 1:
                
                cylinder.color.r = 1;
                cylinder.color.g = 0;
                cylinder.color.b = 0;
                break;
                case 2:

                cylinder.color.r = 0;
                cylinder.color.g = 1;
                cylinder.color.b = 0; 
                break;
                case 3:
                cylinder.color.r = 1;
                cylinder.color.g = 1;
                cylinder.color.b = 0;
                break;
            }

            cylinder.action = visualization_msgs::Marker::ADD;

            cylinder.pose = uavs_poses_[uav_id].pose;
            double diameter=2*(uav_safety_radius_ + bracking_distance_+positioning_error_+ gamma_offset_);
            cylinder.scale.x = diameter;   //Diameter
            cylinder.scale.y = diameter;   // if x e y are different you get an elipse instead o a circle
            cylinder.scale.z = dz_min_;   // height
            cylinder.mesh_use_embedded_materials = true;

            cylinder_markers.markers.push_back(cylinder);

            //publish arrow for goal direction

            if(goal_direction_.find(uav_id) != goal_direction_.end())
            {
                
            
            

                visualization_msgs::Marker arrow_goal;
                arrow_goal.header.frame_id = "/map";
                arrow_goal.header.stamp = ros::Time();
                arrow_goal.id = uav_id;
                arrow_goal.ns = "uavs";
                arrow_goal.type = visualization_msgs::Marker::ARROW;
                arrow_goal.color.a = 1;   
                switch(uav_id)
                {
                    case 1:
                    // orange
                    arrow_goal.color.r = 0.0;
                    arrow_goal.color.g = 0.0;
                    arrow_goal.color.b = 0.5;
                    break;
                    case 2:
                    // ingigo
                    arrow_goal.color.r = 0;
                    arrow_goal.color.g = 0.0;
                    arrow_goal.color.b = 0.5; 
                    break;
                    case 3:
                    // zinc yellow
                    arrow_goal.color.r = 0.0;
                    arrow_goal.color.g = 0.0;
                    arrow_goal.color.b = 0.5;
                    break;
                }

                arrow_goal.action = visualization_msgs::Marker::ADD;
                arrow_goal.points.resize(2);
                arrow_goal.points[0].x=uavs_poses_[uav_id].pose.position.x;
                arrow_goal.points[0].y=uavs_poses_[uav_id].pose.position.y;
                arrow_goal.points[0].z=uavs_poses_[uav_id].pose.position.z;
                arrow_goal.points[1].x=uavs_poses_[uav_id].pose.position.x + goal_direction_[uav_id].x*2;
                arrow_goal.points[1].y=uavs_poses_[uav_id].pose.position.y + goal_direction_[uav_id].y*2;
                arrow_goal.points[1].z= uavs_poses_[uav_id].pose.position.z + goal_direction_[uav_id].z*2; 

            
                arrow_goal.scale.x=0.1;
                arrow_goal.scale.y = 0.2;
            
               // arrow_goal.mesh_use_embedded_materials = true;

                arrow_goal_markers.markers.push_back(arrow_goal);

            }
                  
        
            //publish arrow for avoidance direction

            if(uav_direction_.find(uav_id) != uav_direction_.end())
            {
                
                visualization_msgs::Marker arrow;
                arrow.header.frame_id = "/map";
                arrow.header.stamp = ros::Time();
                arrow.id = uav_id;
                arrow.ns = "uavs";
                arrow.type = visualization_msgs::Marker::ARROW;
                arrow.color.a = 1;   
                switch(uav_id)
                {
                    case 1:
                    // orange
                    arrow.color.r = 0.5;
                    arrow.color.g = 0;
                    arrow.color.b = 0.0;
                    break;
                    case 2:
                    // ingigo
                    arrow.color.r = 0.5;
                    arrow.color.g = 0.0;
                    arrow.color.b = 0.0; 
                    break;
                    case 3:
                    // zinc yellow
                    arrow.color.r = 0.5;
                    arrow.color.g = 0.0;
                    arrow.color.b = 0.0;
                    break;
                }

                arrow.action = visualization_msgs::Marker::ADD;

                
               
                arrow.points.resize(2);
                arrow.points[0].x=uavs_poses_[uav_id].pose.position.x;
                arrow.points[0].y=uavs_poses_[uav_id].pose.position.y;
                arrow.points[0].z=uavs_poses_[uav_id].pose.position.z;
                arrow.points[1].x=uavs_poses_[uav_id].pose.position.x + uav_direction_[uav_id].x*2;
                arrow.points[1].y=uavs_poses_[uav_id].pose.position.y + uav_direction_[uav_id].y*2;
                arrow.points[1].z= uavs_poses_[uav_id].pose.position.z; 
               
                arrow.scale.x=0.1;
                arrow.scale.y = 0.2;
              
                //arrow.mesh_use_embedded_materials = true;

                arrow_markers.markers.push_back(arrow);

            }


        }
    }

    uavs_pub_.publish(uav_markers);

    cylinder_pub_.publish(cylinder_markers);

    arrow_pub_.publish(arrow_markers);

    goal_arrow_pub_.publish(arrow_goal_markers);

    uav_cylinder_pub_.publish(uav_cylinder_markers);

        

}

/** Main function
*/
int main(int argc, char** argv)
{


    ros::init(argc, argv, "visualizer_avoidance_node");
        

    Visualizer vis; 
    
    while(ros::ok())
    {
        ros::spinOnce();

        vis.publishMarkers();
        
        ros::Duration(0.1).sleep();
    }
    
}