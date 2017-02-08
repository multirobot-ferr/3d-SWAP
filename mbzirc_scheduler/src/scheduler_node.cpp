//------------------------------------------------------------------------------
// GRVC MBZIRC
// Author Jesus Capitan <jcapitan@us.es>
// Author Ricardo Ragel
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


#include <mbzirc_scheduler/task_allocator.hpp>
#include <mbzirc_scheduler/centralized_estimator.h>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <mbzirc_scheduler/AssignTarget.h>


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

	/// TODO Callbacks
	//void candidatesReceived(const mbzirc_scheduler::Candidate::ConstPtr& candidate);
	//void uavPoseReceived( );

	void publishBelief();
	bool assignTarget(mbzirc_scheduler::AssignTarget::Request &req, mbzirc_scheduler::AssignTarget::Response &res);

	/// Node handlers
	ros::NodeHandle* nh_; 
	ros::NodeHandle* pnh_;

	/// Estimator frequency
	double estimator_rate_;

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
	pnh_->param<double>("association_th",association_th, 3.0);

	// Estimator and allocator
	estimator_ = new CentralizedEstimator(association_th, lost_time_th);
	allocator_ = new TaskAllocator(estimator_);

	// TODO Subscriptions/publications
	//ros::Subscriber candidates_sub = nh_->subscribe<mbzirc_scheduler::Candidate>("/candidates", 10, &Scheduler::candidatesReceived, this);
	//ros::Subscriber uavs_sub = nh_->subscribe<>("/uav_poses", 10, &Scheduler::uavPoseReceived, this);
	//ros::Publisher belief_markers_pub = nh_->advertise<visualization_msgs::MarkerArray>("/belief", 1);

	// Services
	ros::ServiceServer assign_target_srv = nh_->advertiseService("/scheduler/assign_target", &Scheduler::assignTarget, this);

	// Main loop

	ros::Rate r(estimator_rate_);

	while(nh_->ok())
	{
		ros::spinOnce();

		// TODO predict estimator, publish markers

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
}

/** \brief Callback to receive observations from vision module
*/
// TODO
/*
void Scheduler::candidatesReceived(const mbzirc_scheduler::Candidate::ConstPtr& candidate)
{
	
}
*/

/** \brief Callback to receive UAVs poses
*/
// TODO
/*
void Scheduler::uavPoseReceived(const  uav_pose)
{
	
}
*/

/** \brief Callback for service. Request the assignment of a target
*/
bool Scheduler::assignTarget(mbzirc_scheduler::AssignTarget::Request &req, mbzirc_scheduler::AssignTarget::Response &res)
{
	return true;
}

/** \brief Publish markers to represent targets beliefs
*/
void Scheduler::publishBelief()
{
}

}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "scheduler_node");
  
  mbzirc::Scheduler scheduler; 
  
  return (0);
}

