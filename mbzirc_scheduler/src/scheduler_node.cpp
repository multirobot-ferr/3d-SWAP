//------------------------------------------------------------------------------
// GRVC MBZIRC
// Author Jesus Capitan <jcapitan@us.es>
// Author Ricardo Ragel <delatorre@us.es>
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

#include <mbzirc_scheduler/task_allocator.hpp>
#include <mbzirc_scheduler/centralized_estimator.h>

#include <visualization_msgs/MarkerArray.h>
#include <mbzirc_scheduler/AssignTarget.h>
#include <mbzirc_scheduler/SetTargetStatus.h>
#include <std_msgs/String.h>
#include <uav_state_machine/candidate_list.h>
#include <grvc_quadrotor_hal/types.h>
#include <tf/transform_datatypes.h>

#include<string>
#include<vector>
#include<map>
#include<sstream>

using namespace std;

namespace mbzirc 
{

/** This class implements the Scheduler node with a centralized estimator for target tracking and a task allocator for assigning targets to UAVs.
*/
class Scheduler 
{

public:
	Scheduler();
	~Scheduler();

protected:

	/// Callbacks
	void candidatesReceived(const uav_state_machine::candidate_list::ConstPtr& candidate_list);
	void uavPoseReceived(const std_msgs::String::ConstPtr& uav_pose);

	void publishBelief();
	void eigendec(double c11, double c22, double c12, vector<double> &D, vector<double> &E);
	bool assignTarget(mbzirc_scheduler::AssignTarget::Request &req, mbzirc_scheduler::AssignTarget::Response &res);
	bool setTargetStatus(mbzirc_scheduler::SetTargetStatus::Request &req, mbzirc_scheduler::SetTargetStatus::Response &res);

	/// Node handlers
	ros::NodeHandle* nh_; 
	ros::NodeHandle* pnh_;

	/// Subscribers
	vector<ros::Subscriber *> uav_subs_;
	vector<ros::Subscriber *> candidate_subs_;

	/// Publishers
	ros::Publisher belief_pub_;

	/// Candidates queues
	map<int, vector<Candidate *> > candidates_;

	/// Estimator frequency
	double estimator_rate_;

	/// Maximum delay allowed for candidates (seconds)
	double delay_max_;

	/// Number of UAVs
	int n_uavs_;

	/// Centralized filter for target estimation 
	CentralizedEstimator* estimator_;
	
	/// Task allocator
	TaskAllocator* allocator_;

};

/** \brief Constructor
*/
Scheduler::Scheduler()
{
	nh_ = new ros::NodeHandle();
	pnh_ = new ros::NodeHandle("~");

	double lost_time_th, association_th;
	int task_alloc_mode;
	double task_alloc_alpha;
	

	// Read parameters
	pnh_->param<double>("estimator_rate", estimator_rate_, 5.0); 
	pnh_->param<double>("lost_time_th", lost_time_th, 20.0); 
	pnh_->param<double>("association_th", association_th, 6.0);
	pnh_->param<double>("delay_max", delay_max_, 2.0);
	pnh_->param<int>("task_alloc_mode",task_alloc_mode, LOWER_SCORE_NEAREST);
	pnh_->param<double>("task_alloc_alpha",task_alloc_alpha, 0.8);
	pnh_->param<int>("n_uavs",n_uavs_, 3);

	// Estimator and allocator
	estimator_ = new CentralizedEstimator(association_th, lost_time_th);
	allocator_ = new TaskAllocator(estimator_, (TargetSelectionMode)task_alloc_mode, n_uavs_, task_alloc_alpha);

	// Subscriptions/publications
	for(int i = 0; i < n_uavs_; i++)
	{
		string uav_topic_name = "/mbzirc_" + to_string(i+1) + "/hal/pose";
		string candidate_topic_name = "mbzirc_" + to_string(i+1) + "/candidateList" ;

		ros::Subscriber* candidate_sub = new ros::Subscriber();
		*candidate_sub = nh_->subscribe<uav_state_machine::candidate_list>(candidate_topic_name.c_str(), 1, &Scheduler::candidatesReceived, this);
		candidate_subs_.push_back(candidate_sub);

		ros::Subscriber* uav_sub = new ros::Subscriber();
		*uav_sub = nh_->subscribe<std_msgs::String>(uav_topic_name.c_str(), 1, &Scheduler::uavPoseReceived, this);
		uav_subs_.push_back(uav_sub);

		vector<Candidate *> empty_vector;
		candidates_[i+1] = empty_vector;
	}
	
	belief_pub_ = nh_->advertise<visualization_msgs::MarkerArray>("/targets_belief", 1);

	// Services
	ros::ServiceServer assign_target_srv = nh_->advertiseService("/scheduler/assign_target", &Scheduler::assignTarget, this);
	ros::ServiceServer set_target_status_srv = nh_->advertiseService("/scheduler/set_target_status", &Scheduler::setTargetStatus, this);

	
	// Main loop

	ros::Rate r(estimator_rate_);
	ros::Time prev_time, time_now;

	#ifdef DEBUG_MODE
	int count = 0;
	#endif

	while(nh_->ok())
	{
		ros::spinOnce();

		// Predict estimator
		time_now = ros::Time::now();
		double elapsed_time = (time_now - prev_time).toSec();
		prev_time = time_now;

		if(elapsed_time)
			estimator_->predict(elapsed_time);

		// Update 
		for(auto it = candidates_.begin(); it != candidates_.end(); ++it)
		{
			if((it->second).size())
			{
				#ifdef DEBUG_MODE
				cout << "Candidates from UAV " << it->first << endl;
				#endif 

				estimator_->update(it->second);

				// Remove candidates
				
				for(int j = 0; j < (it->second).size(); j++)
				{
					delete (it->second)[j];
				}
				(it->second).clear();
			}
		}

		estimator_->removeLostTargets();

		publishBelief();

		
		#ifdef DEBUG_MODE
		if(count == 5)
		{
			estimator_->printTargetsInfo();
			count = 0;
		}
		else
			count++;
		#endif

		r.sleep();
	}
}

/** \brief Destructor
*/
Scheduler::~Scheduler()
{
	delete nh_;
	delete pnh_;
	delete estimator_;
	delete allocator_;

	for(int i = 0; i < n_uavs_; i++)
	{
		delete candidate_subs_[i];
		delete uav_subs_[i];

		for(int j = 0; j < candidates_[i+1].size(); j++)
		{
			delete candidates_[i+1][j];
		}
		candidates_[i+1].clear();
	}
	candidates_.clear();
	candidate_subs_.clear();
	uav_subs_.clear();
}

/** \brief Callback to receive observations from vision module
*/
void Scheduler::candidatesReceived(const uav_state_machine::candidate_list::ConstPtr& candidate_list)
{
	double delay = (ros::Time::now() - candidate_list->stamp).toSec();

	if(candidate_list->candidates.size() && delay < delay_max_)
	{
		int uav = candidate_list->uav_id;

		// Remove existing candidates
		if(candidates_[uav].size())
		{
			for(int j = 0; j < candidates_[uav].size(); j++)
			{
				delete candidates_[uav][j];
			}
			candidates_[uav].clear();
		}

		// Store received candidates
		for(int j = 0; j < candidate_list->candidates.size(); j++)
		{
			Candidate* cand_p = new Candidate;

			switch(candidate_list->candidates[j].color)
			{
				case uav_state_machine::candidate::COLOR_UNKNOWN:
				cand_p->color = UNKNOWN;
				break;
				case uav_state_machine::candidate::COLOR_RED:
				cand_p->color = RED;
				break;
				case uav_state_machine::candidate::COLOR_BLUE:
				cand_p->color = BLUE;
				break;
				case uav_state_machine::candidate::COLOR_GREEN:
				cand_p->color = GREEN;
				break;
				case uav_state_machine::candidate::COLOR_YELLOW:
				cand_p->color = YELLOW;
				break;
				case uav_state_machine::candidate::COLOR_ORANGE:
				cand_p->color = ORANGE;
				break;
				default:
				ROS_ERROR("Invalid candidate color received.");
			}

			cand_p->shape = candidate_list->candidates[j].shape;
			cand_p->height = candidate_list->candidates[j].height;
			cand_p->width = candidate_list->candidates[j].width;

			cand_p->location(0) = candidate_list->candidates[j].global_position.x;
			cand_p->location(1) = candidate_list->candidates[j].global_position.y;
			cand_p->location(2) = candidate_list->candidates[j].global_position.z;

			for(int i = 0; i < 3; i++)
			{
				for(int k = 0; k < 3; k++)
				{
					cand_p->locationCovariance(i,k) = candidate_list->candidates[j].position_covariance[i*3+k];
				}	
			}			

			candidates_[uav].push_back(cand_p); 
		}
	}
	else
		ROS_WARN("Received candidates empty or with long delay.");
}

/** \brief Callback to receive UAVs poses
*/
void Scheduler::uavPoseReceived(const std_msgs::String::ConstPtr& uav_pose)
{
	stringstream msg;
	msg << uav_pose->data;
	grvc::hal::Pose pose;
	msg >> pose;
	int uav_id = stoi(pose.id);

	if(0 < uav_id && uav_id <= n_uavs_)
		allocator_->updateUavPosition(uav_id, pose.position[0], pose.position[1], pose.position[2]);
}

/** \brief Callback for service. Request the assignment of a target
*/
bool Scheduler::assignTarget(mbzirc_scheduler::AssignTarget::Request &req, mbzirc_scheduler::AssignTarget::Response &res)
{	
	bool result;
	double x, y;
	TargetStatus target_status;
	Color target_color;

	if(0 < req.uav_id && req.uav_id <= n_uavs_)
	{
		res.target_id = allocator_->getOptimalTarget(req.uav_id);

		estimator_->getTargetInfo(res.target_id, x, y, target_status, target_color);

		res.global_position.x = x;
		res.global_position.y = y;
		res.global_position.z = 0.0;

		switch(target_color)
		{
			case UNKNOWN:
			res.color = uav_state_machine::candidate::COLOR_UNKNOWN;
			break;
			case RED:
			res.color = uav_state_machine::candidate::COLOR_RED;
			break;
			case BLUE:
			res.color = uav_state_machine::candidate::COLOR_BLUE;
			break;
			case GREEN:
			res.color = uav_state_machine::candidate::COLOR_GREEN;
			break;
			case YELLOW:
			res.color = uav_state_machine::candidate::COLOR_YELLOW;
			break;
			case ORANGE:
			res.color = uav_state_machine::candidate::COLOR_ORANGE;
			break;
		}

		// Set target to assigned
		estimator_->setTargetStatus(res.target_id, ASSIGNED);

		result = true;
	}
	else
		result = false;

	return result;
}

/** \brief Callback for service. Set the status of a target
*/
bool Scheduler::setTargetStatus(mbzirc_scheduler::SetTargetStatus::Request &req, mbzirc_scheduler::SetTargetStatus::Response &res)
{
	TargetStatus target_status;

	switch(req.target_status)
	{
		case mbzirc_scheduler::SetTargetStatus::Request::UNASSIGNED:
		target_status = UNASSIGNED;
		break;
		case mbzirc_scheduler::SetTargetStatus::Request::ASSIGNED:
		target_status = ASSIGNED;
		break;
		case mbzirc_scheduler::SetTargetStatus::Request::CAUGHT:
		target_status = CAUGHT;
		break;
		case mbzirc_scheduler::SetTargetStatus::Request::DEPLOYED:
		target_status = DEPLOYED;
		break;
		case mbzirc_scheduler::SetTargetStatus::Request::LOST:
		target_status = LOST;
		break;
		default:
		ROS_ERROR("Not valid target status for assignment.");
	}	

	return estimator_->setTargetStatus(req.target_id, target_status);
}

/** \brief Publish markers to represent targets beliefs
*/
void Scheduler::publishBelief()
{
	visualization_msgs::MarkerArray marker_array;
	ros::Time curr_time = ros::Time::now();

	vector<double> w(2);
	vector<double> v(4);
	vector<vector<double> > covariances;
	double a, b, yaw, x, y, vx, vy;
	TargetStatus target_status;
	Color target_color;

	// Get ids to plot active targets
	vector<int> active_targets = estimator_->getActiveTargets();

	for(int i = 0; i < active_targets.size(); i++)
	{
		if(estimator_->getTargetInfo(active_targets[i], x, y, covariances, vx, vy))
		{
			estimator_->getTargetInfo(active_targets[i], x, y, target_status, target_color);

			// Compute SVD of cholesky. The singular values are the square roots of the eigenvalues of
			// the covariance matrix 
			eigendec(4*covariances[0][0], 4*covariances[1][1], 4*covariances[0][1], w, v);

			a = sqrt(fabs(w[0]));
			b = sqrt(fabs(w[1]));

			yaw = atan2(v[1],v[0]);
			
			// Fill in marker
			visualization_msgs::Marker marker;

			// Set color for the target, default if UNKNOWN
			switch(target_color)
			{
				case RED:
				marker.color.r = 1.0;
				marker.color.g = 0.0;
				marker.color.b = 0.0;
				marker.color.a = 1;
				break;
				case BLUE:
				marker.color.r = 0.0;
				marker.color.g = 0.0;
				marker.color.b = 1.0;
				marker.color.a = 1;
				break;
				case GREEN:
				marker.color.r = 0.0;
				marker.color.g = 1.0;
				marker.color.b = 0.0;
				marker.color.a = 1;
				break;
				case YELLOW:
				marker.color.r = 1.0;
				marker.color.g = 1.0;
				marker.color.b = 0.0;
				marker.color.a = 1;
				break;
				case ORANGE:
				marker.color.r = 1.0;
				marker.color.g = 0.65;
				marker.color.b = 0.0;
				marker.color.a = 1;
				break;
				default:
				marker.color.r = 0.0;
				marker.color.g = 0.0;
				marker.color.b = 0.0;
				marker.color.a = 1;
				break;
			}
		
			// Set the frame ID and timestamp
			marker.header.frame_id = "/map";    
			marker.header.stamp = curr_time;

			// Set the namespace and id for this marker.  This serves to create a unique ID    
			// Any marker sent with the same namespace and id will overwrite the old one    
			marker.ns = "cov_ellipse";    
			marker.id = i;
		
			// Set the marker type    
			marker.type = visualization_msgs::Marker::SPHERE;

			// Set the marker action.  Options are ADD and DELETE    
			marker.action = visualization_msgs::Marker::ADD;
			
			// Set the scale of the marker -- 1x1x1 here means 1m on a side
			marker.scale.x = a;
			marker.scale.y = b;    
			marker.scale.z = 0.1;

			marker.lifetime = ros::Duration();

			// Set the central pose of the marker. This is a full 6DOF pose relative to the frame/time specified in the header    
			marker.pose.position.x = x;
			marker.pose.position.y = y;
			marker.pose.position.z = 0;
			marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

			marker_array.markers.push_back(marker);

			// Plot velocity
			marker.ns = "velocity";
			marker.type = visualization_msgs::Marker::ARROW;    
			marker.scale.x = sqrt(vx*vx+vy*vy);
			marker.scale.y = 0.1;    
			marker.scale.z = 0.1;
			
			marker_array.markers.push_back(marker);

			// Plot target ID
			marker.ns = "target_id";
			marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
			marker.text = to_string(active_targets[i]);
			marker.pose.position.x += 1.0;
			marker.pose.position.y += 1.0;    
			marker.scale.z = 1.0;
			
			marker_array.markers.push_back(marker);
		}
		else
			ROS_ERROR("Target ID not found");
	}
	// Publish the marker    
	belief_pub_.publish(marker_array);
}

/**
 Closed form eigenvalue decomposition
 
 C  Positive definite input matrix of form
        I[0] I[1]
        I[2] I[3]   where I[1]=I[2]
 D  Output eigenvalues
 E  Eigenvector matrix of form
        E[0] E[2]
        E[1] E[3]
*/
void Scheduler::eigendec(double c11, double c22, double c12, vector<double> &D, vector<double> &E)
{ 
	double a,b,enorm;

	// Find eigenvalues 
	a = c11+c22;                         // trace 
 	b = sqrt((c11-c22)*(c11-c22)+4*c12*c12);
 	D[0] = (a+b)/2;
 	D[1] = (a-b)/2;

	// Find eigenvector 1 
	E[0] = c22+c12-D[0];
	E[1] = D[0]-c11-c12;
	enorm = sqrt(E[0]*E[0]+E[1]*E[1]);

	if(enorm > 0.0) {
		E[0] = E[0]/enorm;
		E[1] = E[1]/enorm;
	}

	// Find eigenvector 2 
	E[2] = c22+c12-D[1];
	E[3] = D[1]-c11-c12;
	enorm = sqrt(E[2]*E[2]+E[3]*E[3]);

	if(enorm > 0.0) {
		E[2] = E[2]/enorm;
		E[3] = E[3]/enorm;
	}
}

}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "scheduler_node");
  
  mbzirc::Scheduler scheduler; 
  
  return (0);
}

