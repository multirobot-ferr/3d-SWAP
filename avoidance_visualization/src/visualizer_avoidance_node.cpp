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
#include <iomanip> // setprecision

using namespace std;

/** This class subscribes to avoidance_swap data to publish visual marker
*/

class Visualizer
{

public:
    Visualizer(int id);
    ~Visualizer();

    void publishMarkers();
    
protected:

        /// Callbacks

        void uavPoseReceived(const geometry_msgs::PoseStamped::ConstPtr& uav_pose);

        void AvoidMovementCallback(const geometry_msgs::Vector3::ConstPtr& avoidance_direction);

        void GoalDirectionCallback(const geometry_msgs::Vector3::ConstPtr& goal_direction);

        ///Node handlers
        
        /*ros::NodeHandle* nh_;
        ros::NodeHandle* pnh_; */
        ros::NodeHandle nh_; 
        ros::NodeHandle* pnh_;                  //!< Private node handler


        /// Subscribers

        ros::Subscriber uav_subs_;
        ros::Subscriber direction_subs_;
        ros::Subscriber goal_direction_subs_;


        /// Publishers
        ros::Publisher scenario_pub_;          // Publisher for scenario marker
        ros::Publisher cylinder_pub_;          // Publisher for cylinder marker
        ros::Publisher uavs_pub_;              // Publisher for uav pose
        ros::Publisher arrow_pub_;             // Publisher for avoidance direction marker
        ros::Publisher goal_arrow_pub_;        // Publisher for goal direction marker
        ros::Publisher uav_cylinder_pub_;       // Publisher for safety cylinder
        ros::Publisher height_marker_pub_;
        ros::Publisher id_marker_pub_; 
        ros::Publisher goal_marker_pub_;
       

        geometry_msgs::PoseStamped uavs_poses_;
        geometry_msgs::Vector3 uav_direction_;
        bool uav_direction_received_;
        geometry_msgs::Vector3 goal_direction_;
        bool uav_position_received_;


        int uav_id;

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
Visualizer::Visualizer(int id)
{

    pnh_ = new ros::NodeHandle("~");
    
    // getting swap parameters

    cout<<"el id es: "<<id<<endl;

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

        uav_id=id;
    // Subscriptions/publications


        string uav_topic_name = "uav_" + to_string(id) + "/ual/pose";
        string uav_direction_topic_name =  "uav_" + to_string(id) + "/avoid_movement_direction";
        string goal_direction_topic_name= "uav_" + to_string(id) + "/wished_movement_direction";


        uav_subs_ = nh_.subscribe<geometry_msgs::PoseStamped>(uav_topic_name.c_str(), 1, &Visualizer::uavPoseReceived, this);

        direction_subs_ = nh_.subscribe<geometry_msgs::Vector3>(uav_direction_topic_name.c_str(), 1, &Visualizer::AvoidMovementCallback, this);

        goal_direction_subs_ = nh_.subscribe<geometry_msgs::Vector3>(goal_direction_topic_name.c_str(), 1, &Visualizer::GoalDirectionCallback, this);

        goal_marker_pub_= nh_.advertise<visualization_msgs::Marker>("uav_" + std::to_string(id) + "/goal",0);
        id_marker_pub_= nh_.advertise<visualization_msgs::Marker>("uav_" + std::to_string(id) + "/id",0);
        height_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("uav_" + std::to_string(id) + "/height", 0);
        uavs_pub_ = nh_.advertise<visualization_msgs::Marker>("uav_" + std::to_string(id) + "/uavs", 0);
        cylinder_pub_= nh_.advertise<visualization_msgs::Marker>("uav_" + std::to_string(id) + "/bracking_cylinder",0);
        arrow_pub_= nh_.advertise<visualization_msgs::Marker>("uav_" + std::to_string(id) + "/arrow",0);
        goal_arrow_pub_ = nh_.advertise<visualization_msgs::Marker>("uav_" + std::to_string(id) + "/arrow_goal",0);
        uav_cylinder_pub_ = nh_.advertise<visualization_msgs::Marker>("uav_" + std::to_string(id) + "/safety_cylinder", 0);
}
/** \brief Destructor
*/
Visualizer::~Visualizer()
    {
    
    }

/** \brief Callback to receive UAVs poses
*/
void Visualizer::uavPoseReceived(const geometry_msgs::PoseStamped::ConstPtr& uav_pose)
{
        uavs_poses_ = *uav_pose;
}

/** \brief Callback for avoid direction
*/

void Visualizer::AvoidMovementCallback(const geometry_msgs::Vector3::ConstPtr& avoidance_direction)
{
    // Receives an avoidance direction (when is necessary to take one)
    uav_direction_ = *avoidance_direction;
    uav_direction_received_ = true;
}

void Visualizer::GoalDirectionCallback(const geometry_msgs::Vector3::ConstPtr& goal_direction)
{
    goal_direction_ = *goal_direction;
}

/** Publish markers
*/
void Visualizer::publishMarkers()
{

  

    // Publish UAVs and cylinder
    

    //uav_marker
    // If not empty (never received)
        

            visualization_msgs::Marker height_marker;
            height_marker.header.frame_id = "/map";
            height_marker.header.stamp = ros::Time();
            height_marker.id = uav_id;
            height_marker.ns = "uavs_state";
            height_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            stringstream stream;
            stream << fixed << setprecision(2) << uavs_poses_.pose.position.z;
            string s= stream.str();
            height_marker.text = s;
            height_marker.pose.position.z = uavs_poses_.pose.position.z+2.0;
            height_marker.pose.position.y = uavs_poses_.pose.position.y+1;
            height_marker.pose.position.x = uavs_poses_.pose.position.x;
            height_marker.color.a=1;
            height_marker.color.r=1;
            height_marker.color.b=1;
            height_marker.color.g=1;
            height_marker.scale.x = 0.5;
            height_marker.scale.y = 0.5;
            height_marker.scale.z = 0.5;
            height_marker.mesh_use_embedded_materials = true;

       
            visualization_msgs::Marker marker_robot;
            marker_robot.header.frame_id = "/map";
            marker_robot.header.stamp = ros::Time();
            marker_robot.id = uav_id;
            marker_robot.ns = "uavs";
            marker_robot.type = visualization_msgs::Marker::MESH_RESOURCE;
            marker_robot.mesh_resource = "package://robots_description/models/mbzirc/meshes/multirotor.dae";
            marker_robot.color.a = 1;    
            marker_robot.action = visualization_msgs::Marker::ADD;

            marker_robot.pose = uavs_poses_.pose;

            marker_robot.scale.x = 0.001;
            marker_robot.scale.y = 0.001;
            marker_robot.scale.z = 0.001;
            marker_robot.mesh_use_embedded_materials = true;

            switch(uav_id)
            {
                case 1:
                // orange
                marker_robot.color.r = 1.0;
                marker_robot.color.g = 0.647;
                marker_robot.color.b = 0.0;
                break;
                case 2:
                // ingigo
                marker_robot.color.r = 0.294;
                marker_robot.color.g = 0.0;
                marker_robot.color.b = 0.510; 
                break;
                case 3:
                // zinc yellow
                marker_robot.color.r = 0.945;
                marker_robot.color.g = 0.812;
                marker_robot.color.b = 0.267;
                break;
            }



            visualization_msgs::Marker id_marker;
            id_marker.header.frame_id = "/map";
            id_marker.header.stamp = ros::Time();
            id_marker.id = uav_id;
            id_marker.ns = "uavs_state";
            id_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            id_marker.text = std::to_string(uav_id);
            id_marker.pose.position.z = uavs_poses_.pose.position.z+2.0;
            id_marker.pose.position.y = uavs_poses_.pose.position.y;
            id_marker.pose.position.x = uavs_poses_.pose.position.x;
            id_marker.color.a=1;
             switch(uav_id)
            {
                case 1:
                // orange
                id_marker.color.r = 1.0;
                id_marker.color.g = 0.647;
                id_marker.color.b = 0.0;
                break;
                case 2:
                // ingigo
                id_marker.color.r = 0.294;
                id_marker.color.g = 0.0;
                id_marker.color.b = 0.510; 
                break;
                case 3:
                // zinc yellow
                id_marker.color.r = 0.945;
                id_marker.color.g = 0.812;
                id_marker.color.b = 0.267;
                break;
            }
            id_marker.scale.x = 1;
            id_marker.scale.y = 1;
            id_marker.scale.z = 1;
            id_marker.mesh_use_embedded_materials = true;

            /*
            marker.ns = "uavs_state";
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.text = std::to_string(uav_id) + " ";
            marker.pose.position.z += 2.0;
            
            marker.scale.x = 1;
            marker.scale.y = 1;
            marker.scale.z = 1;
            
            */
            // Publish uav cylinder

            visualization_msgs::Marker goal_marker;
            goal_marker.header.frame_id = "/map";
            goal_marker.header.stamp = ros::Time();
            goal_marker.id = uav_id;
            goal_marker.ns = "uavs_state";
            goal_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            goal_marker.text = std::to_string(uav_id);
            goal_marker.pose.position.z = uavs_poses_.pose.position.z+2.0;
            goal_marker.pose.position.y = uavs_poses_.pose.position.y;
            goal_marker.pose.position.x = uavs_poses_.pose.position.x;
            goal_marker.color.a=1;
             switch(uav_id)
            {
                case 1:
                // orange
                goal_marker.color.r = 1.0;
                goal_marker.color.g = 0.647;
                goal_marker.color.b = 0.0;
                break;
                case 2:
                // ingigo
                goal_marker.color.r = 0.294;
                goal_marker.color.g = 0.0;
                goal_marker.color.b = 0.510; 
                break;
                case 3:
                // zinc yellow
                goal_marker.color.r = 0.945;
                goal_marker.color.g = 0.812;
                goal_marker.color.b = 0.267;
                break;
            }
            goal_marker.scale.x = 1;
            goal_marker.scale.y = 1;
            goal_marker.scale.z = 1;
            goal_marker.mesh_use_embedded_materials = true;



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

            uav_cylinder.pose.position = uavs_poses_.pose.position;
            uav_cylinder.scale.x = 2*(uav_safety_radius_ + positioning_error_);   //Diameter
            uav_cylinder.scale.y = 2*(uav_safety_radius_ + positioning_error_);   // if x e y are different you get an elipse instead o a circle
            uav_cylinder.scale.z = dz_min_;   // height
            uav_cylinder.mesh_use_embedded_materials = true;


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
                    if(uav_direction_received_==false){
                    cylinder.color.r = 0;
                    cylinder.color.g = 1;
                    cylinder.color.b = 0;
                    }
                    else
                    {
                    cylinder.color.r = 1;
                    cylinder.color.g = 1;
                    cylinder.color.b = 0;
                    }

                break;
                case 2:

                if(uav_direction_received_==false){
                    cylinder.color.r = 0;
                    cylinder.color.g = 1;
                    cylinder.color.b = 0;
                    }
                    else
                    {
                    cylinder.color.r = 1;
                    cylinder.color.g = 1;
                    cylinder.color.b = 0;
                    }
                break;
                case 3:
                if(uav_direction_received_==false){
                    cylinder.color.r = 0;
                    cylinder.color.g = 1;
                    cylinder.color.b = 0;
                    }
                    else
                    {
                    cylinder.color.r = 1;
                    cylinder.color.g = 1;
                    cylinder.color.b = 0;
                    }
                break;
            }

            cylinder.action = visualization_msgs::Marker::ADD;

            cylinder.pose.position = uavs_poses_.pose.position;
            double diameter=2*(uav_safety_radius_ + bracking_distance_+positioning_error_+ gamma_offset_);
            cylinder.scale.x = diameter;   //Diameter
            cylinder.scale.y = diameter;   // if x e y are different you get an elipse instead o a circle
            cylinder.scale.z = dz_min_;   // height
            cylinder.mesh_use_embedded_materials = true;


            //publish arrow for goal direction

            

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

                double module= sqrt(powf(goal_direction_.x, 2.0) + powf(goal_direction_.y, 2.0) + powf(goal_direction_.z, 2.0));
                arrow_goal.action = visualization_msgs::Marker::ADD;
                arrow_goal.points.resize(2);
                arrow_goal.points[0].x=uavs_poses_.pose.position.x;
                arrow_goal.points[0].y=uavs_poses_.pose.position.y;
                arrow_goal.points[0].z=uavs_poses_.pose.position.z;
                arrow_goal.points[1].x=uavs_poses_.pose.position.x + (goal_direction_.x*2/module);
                arrow_goal.points[1].y=uavs_poses_.pose.position.y + (goal_direction_.y*2/module);
                arrow_goal.points[1].z= uavs_poses_.pose.position.z + (goal_direction_.z*2/module); 

            
                arrow_goal.scale.x=0.1;
                arrow_goal.scale.y = 0.2;
            
               // arrow_goal.mesh_use_embedded_materials = true;


            
                  
        
            //publish arrow for avoidance direction

            
            
                
                visualization_msgs::Marker arrow;
                arrow.header.frame_id = "/map";
                arrow.header.stamp = ros::Time();
                arrow.id = uav_id;
                arrow.ns = "uavs";
                arrow.type = visualization_msgs::Marker::ARROW;
                arrow.lifetime = ros::Duration(0.15);
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
                arrow.points[0].x=uavs_poses_.pose.position.x;
                arrow.points[0].y=uavs_poses_.pose.position.y;
                arrow.points[0].z=uavs_poses_.pose.position.z;
                arrow.points[1].x=uavs_poses_.pose.position.x + uav_direction_.x*2;
                arrow.points[1].y=uavs_poses_.pose.position.y + uav_direction_.y*2;
                arrow.points[1].z= uavs_poses_.pose.position.z; 
               
                arrow.scale.x=0.1;
                arrow.scale.y = 0.2;
              
                //arrow.mesh_use_embedded_materials = true;

                uav_direction_received_ = false;
                

        


        
    

    uavs_pub_.publish(marker_robot);

    cylinder_pub_.publish(cylinder);

    arrow_pub_.publish(arrow);

    goal_arrow_pub_.publish(arrow_goal);

    uav_cylinder_pub_.publish(uav_cylinder);

    height_marker_pub_.publish(height_marker);

    id_marker_pub_.publish(id_marker);

    //goal_marker_pub_.publish(id_marker);
    //
}


/** Main function
*/
int main(int argc, char** argv)
{

    ros::init(argc, argv, "visualizer_avoidance_node");
        
    vector<Visualizer> robot;    

   /* for(int i=1; i<=3; i++){

        robot.push_back(Visualizer(i));
    }*/
    Visualizer vis_uav1(1);

    Visualizer vis_uav2(2);

    Visualizer vis_uav3(3);
    
    while(ros::ok())
    {
        ros::spinOnce();


   /* for(int i=0; i<3; i++){

      robot[i].publishMarkers();
          
    } */

        vis_uav1.publishMarkers();

        vis_uav2.publishMarkers();

        vis_uav3.publishMarkers();
        
        ros::Duration(0.1).sleep();
    }
    
}
