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
#include <geometry_msgs/PoseStamped.h>
#include <uav_state_machine/candidate_list.h>

#include<string>
#include<vector>
#include<map>

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
	void uavPoseReceived(const geometry_msgs::PoseStamped::ConstPtr& uav_pose);

	void publishBelief();
	bool assignTarget(mbzirc_scheduler::AssignTarget::Request &req, mbzirc_scheduler::AssignTarget::Response &res);

	/// Node handlers
	ros::NodeHandle* nh_; 
	ros::NodeHandle* pnh_;

	/// Subscribers
	vector<ros::Subscriber *> uav_subs_;
	vector<ros::Subscriber *> candidate_subs_;

	/// Candidates queues
	map<int, vector<Candidate *> > candidates_;

	/// Estimator frequency
	double estimator_rate_;

	/// Maximum delay allowed for candidates (seconds)
	double delay_max_;

	/// Task Allocator selection mode {NEAREST = 1, LOWER_SCORE_NEAREST = 2, WEIGHTED_SCORE_AND_DISTANCE = 3}
	int task_alloc_mode;

	/// Task Allocator coefficient beetween Distance from UAV and Score/Difficulty 
	/// Only needed if WEIGHTED_SCORE_AND_DISTANCE target selection mode.
	double task_alloc_alpha;

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

	double lost_time_th;
	double association_th;

	// Read parameters
	pnh_->param<double>("estimator_rate", estimator_rate_, 5.0); 
	pnh_->param<double>("lost_time_th", lost_time_th, 20.0); 
	pnh_->param<double>("association_th", association_th, 3.0);
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
		string uav_topic_name = "/mavros_" + to_string(i+1) + "/local_position/pose";
		string candidate_topic_name = "/candidate_list_" + to_string(i+1);

		ros::Subscriber* candidate_sub = new ros::Subscriber();
		*candidate_sub = nh_->subscribe<uav_state_machine::candidate_list>(candidate_topic_name.c_str(), 1, &Scheduler::candidatesReceived, this);
		candidate_subs_.push_back(candidate_sub);

		ros::Subscriber* uav_sub = new ros::Subscriber();
		*uav_sub = nh_->subscribe<geometry_msgs::PoseStamped>(uav_topic_name.c_str(), 1, &Scheduler::uavPoseReceived, this);
		uav_subs_.push_back(uav_sub);

		vector<Candidate *> empty_vector;
		candidates_[i+1] = empty_vector;
	}
	
	ros::Publisher belief_markers_pub = nh_->advertise<visualization_msgs::MarkerArray>("/belief", 1);

	// Services
	ros::ServiceServer assign_target_srv = nh_->advertiseService("/scheduler/assign_target", &Scheduler::assignTarget, this);

	// Main loop

	ros::Rate r(estimator_rate_);
	ros::Time prev_time, time_now;

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
				estimator_->update(it->second);
		}

		estimator_->removeLostTargets();

		publishBelief();

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

			cand_p->color = candidate_list->candidates[j].color;
			cand_p->shape = candidate_list->candidates[j].shape;
			cand_p->height = candidate_list->candidates[j].height;
			cand_p->width = candidate_list->candidates[j].width;

			cand_p->location(0) = candidate_list->candidates[j].position.x;
			cand_p->location(1) = candidate_list->candidates[j].position.y;
			cand_p->location(1) = candidate_list->candidates[j].position.z;

			for(int i = 0; i < 3; i++)
			{
				for(int k = 0; k < 3; k++)
				{
					cand_p->locationCovariance(i,k) = candidate_list->candidates[j].position_covariance[i*3+j];
				}	
			}			

			candidates_[uav].push_back(cand_p); 
		}
	}
}

/** \brief Callback to receive UAVs poses
*/
void Scheduler::uavPoseReceived(const geometry_msgs::PoseStamped::ConstPtr& uav_pose)
{
	allocator_->updateUavPosition(stoi(uav_pose->header.frame_id), uav_pose->pose.position.x, uav_pose->pose.position.y, uav_pose->pose.position.z);
}

/** \brief Callback for service. Request the assignment of a target
*/
bool Scheduler::assignTarget(mbzirc_scheduler::AssignTarget::Request &req, mbzirc_scheduler::AssignTarget::Response &res)
{	
	res.target_id = allocator_->getOptimalTarget(req.uav_id);

	return true;
}

/** \brief Publish markers to represent targets beliefs
*/
void Scheduler::publishBelief()
{
	// TODO
}

}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "scheduler_node");
  
  mbzirc::Scheduler scheduler; 
  
  return (0);
}

