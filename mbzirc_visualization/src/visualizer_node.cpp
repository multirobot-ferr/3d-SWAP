//------------------------------------------------------------------------------
// GRVC MBZIRC
// Author Jesus Capitan <jcapitan@us.es>
// 
//------------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2016 GRVC University of Seville
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

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <uav_state_machine/CandidateList.h>
#include <uav_state_machine/UavState.h>

#include<string>
#include<vector>
#include<sstream>

using namespace std;

namespace mbzirc 
{

/** This class subscribes to MBZIRC data to publish visual markers for rViz.
*/
class Visualizer 
{

public:
	Visualizer();
	~Visualizer();

    void publishMarkers();
	
protected:

	/// Callbacks
	void candidatesReceived(const uav_state_machine::CandidateList::ConstPtr& candidate_list);
	void uavPoseReceived(const geometry_msgs::PoseStamped::ConstPtr& uav_pose);
    void uavStateReceived(const uav_state_machine::UavState::ConstPtr& uav_state);

	/// Node handlers
	ros::NodeHandle* nh_; 
	ros::NodeHandle* pnh_;

	/// Subscribers
	vector<ros::Subscriber *> uav_subs_;
	vector<ros::Subscriber *> candidate_subs_;
    vector<ros::Subscriber *> uav_state_subs_;

	/// Publishers
	ros::Publisher scenario_pub_;
    ros::Publisher uavs_pub_;
    ros::Publisher uavs_state_pub_;
    ros::Publisher candidates_pub_;

	/// Last data received
	map<int, uav_state_machine::CandidateList> candidates_;
    map<int, geometry_msgs::PoseStamped> uavs_poses_;
    map<int, std::string> uavs_states_;

	/// Number of UAVs
	int n_uavs_;

};

/** \brief Constructor
*/
Visualizer::Visualizer()
{
	nh_ = new ros::NodeHandle();
	pnh_ = new ros::NodeHandle("~");

	// Read parameters
	pnh_->param<int>("n_uavs",n_uavs_, 3);

	// Subscriptions/publications
	for(int i = 0; i < n_uavs_; i++)
	{
		string uav_topic_name = "ual_" + to_string(i+1) + "/pose";
		string candidate_topic_name = "mbzirc_" + to_string(i+1) + "/candidateList";
        string uav_state_topic_name = "mbzirc_" + to_string(i+1) + "/uav_state_machine/state" ;

		ros::Subscriber* candidate_sub = new ros::Subscriber();
		*candidate_sub = nh_->subscribe<uav_state_machine::CandidateList>(candidate_topic_name.c_str(), 1, &Visualizer::candidatesReceived, this);
		candidate_subs_.push_back(candidate_sub);

		ros::Subscriber* uav_sub = new ros::Subscriber();
		*uav_sub = nh_->subscribe<geometry_msgs::PoseStamped>(uav_topic_name.c_str(), 1, &Visualizer::uavPoseReceived, this);
		uav_subs_.push_back(uav_sub);

        ros::Subscriber* uav_state_sub = new ros::Subscriber();
		*uav_state_sub = nh_->subscribe<uav_state_machine::UavState>(uav_state_topic_name.c_str(), 1, &Visualizer::uavStateReceived, this);
		uav_state_subs_.push_back(uav_state_sub);
	}
	
    scenario_pub_ = nh_->advertise<visualization_msgs::Marker>("mbzirc_markers/scenario", 0);
    uavs_pub_ = nh_->advertise<visualization_msgs::MarkerArray>("mbzirc_markers/uavs", 0);
    uavs_state_pub_ = nh_->advertise<visualization_msgs::MarkerArray>("mbzirc_markers/uavs_states", 0);
    candidates_pub_ = nh_->advertise<visualization_msgs::MarkerArray>("mbzirc_markers/candidates", 1);
}

/** \brief Destructor
*/
Visualizer::~Visualizer()
{
	delete nh_;
	delete pnh_;

	for(int i = 0; i < n_uavs_; i++)
	{
		delete candidate_subs_[i];
		delete uav_subs_[i];
        delete uav_state_subs_[i];
	}
	candidates_.clear();
    uavs_poses_.clear();
	candidate_subs_.clear();
	uav_subs_.clear();
    uav_state_subs_.clear();
}

/** \brief Callback to receive observations from vision module
*/
void Visualizer::candidatesReceived(const uav_state_machine::CandidateList::ConstPtr& candidate_list)
{
    int uav_id = candidate_list->uav_id;
    if(0 < uav_id && uav_id <= n_uavs_)
    {
        uav_state_machine::CandidateList candidates = *candidate_list;
        candidates_[uav_id] = candidates;
    }
}

/** \brief Callback to receive UAVs poses
*/
void Visualizer::uavPoseReceived(const geometry_msgs::PoseStamped::ConstPtr& uav_pose)
{
    int uav_id; // TODO get uav_id
	//int uav_id = stoi(pose.id);

	if(0 < uav_id && uav_id <= n_uavs_)
        uavs_poses_[uav_id] = *uav_pose;
}

/** \brief Callback to receive UAVs states
*/
void Visualizer::uavStateReceived(const uav_state_machine::UavState::ConstPtr& uav_state)
{
	if(0 < uav_state->uav_id && uav_state->uav_id <= n_uavs_)
    {
        uavs_states_[uav_state->uav_id] = uav_state->state_str.data + std::string(" ") + uav_state->state_msg.data;
    }
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

    // Publish UAVs and states
    visualization_msgs::MarkerArray uav_markers;
    visualization_msgs::MarkerArray uav_state_markers;

    for (int uav_id = 1; uav_id <= n_uavs_; uav_id++)
    {
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
            marker.text = std::to_string(uav_id) + " " + uavs_states_[uav_id];
            marker.pose.position.z += 2.0;
            
            marker.scale.x = 1;
            marker.scale.y = 1;
            marker.scale.z = 1;
            
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;

            uav_state_markers.markers.push_back(marker);
        }
    }

    uavs_pub_.publish(uav_markers);
    uavs_state_pub_.publish(uav_state_markers);

    // Publish candidates
    visualization_msgs::MarkerArray candidate_markers;

    for (int uav_id = 1; uav_id <= n_uavs_; uav_id++)
    {
        // If not empty (never received)
        auto it = candidates_.find(uav_id);
        if(it != candidates_.end())
        {
            for(unsigned int i = 0; i < candidates_[uav_id].candidates.size(); i++)
            {
                visualization_msgs::Marker marker;
                marker.header.frame_id = "/map";
                marker.header.stamp = ros::Time();
                marker.id = i;
                marker.ns = "uav_" + to_string(uav_id);
                marker.type = visualization_msgs::Marker::CUBE;
                marker.lifetime = ros::Duration(1.0);

                // Set color for the target, default if UNKNOWN
                switch(candidates_[uav_id].candidates[i].color)
                {
                    case uav_state_machine::Candidate::COLOR_RED:
                    marker.color.r = 1.0;
                    marker.color.g = 0.0;
                    marker.color.b = 0.0;
                    break;
                    case uav_state_machine::Candidate::COLOR_BLUE:
                    marker.color.r = 0.0;
                    marker.color.g = 0.0;
                    marker.color.b = 1.0;
                    break;
                    case uav_state_machine::Candidate::COLOR_GREEN:
                    marker.color.r = 0.0;
                    marker.color.g = 1.0;
                    marker.color.b = 0.0;
                    break;
                    case uav_state_machine::Candidate::COLOR_YELLOW:
                    marker.color.r = 1.0;
                    marker.color.g = 1.0;
                    marker.color.b = 0.0;
                    break;
                    case uav_state_machine::Candidate::COLOR_ORANGE:
                    marker.color.r = 1.0;
                    marker.color.g = 0.65;
                    marker.color.b = 0.0;
                    break;
                    default:
                    marker.color.r = 0.0;
                    marker.color.g = 0.0;
                    marker.color.b = 0.0;
                    break;
                }

                marker.color.a = 1;    
                marker.action = visualization_msgs::Marker::ADD;

                marker.pose.position.x = candidates_[uav_id].candidates[i].global_position.x;
                marker.pose.position.y = candidates_[uav_id].candidates[i].global_position.y;
                marker.pose.position.z = 0.0;

                if(uavs_poses_.find(uav_id) != uavs_poses_.end())
                {
                    marker.pose.orientation = uavs_poses_[uav_id].pose.orientation;
                }

                marker.scale.x = 0.5;
                marker.scale.y = 0.5;
                marker.scale.z = 0.05;
                
                candidate_markers.markers.push_back(marker);
            }

            candidates_.erase(it);
        }
    }

    candidates_pub_.publish(candidate_markers);
}

}

/** Main function
*/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "visualizer_node");
  
    mbzirc::Visualizer vis; 

    ros::Rate loop(30);

    while(ros::ok())
    {
        ros::spinOnce();
        vis.publishMarkers();
        loop.sleep();
    }
}