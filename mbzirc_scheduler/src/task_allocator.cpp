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


#include <mbzirc_scheduler/task_allocator.hpp>

namespace mbzirc {

/** Constructor
\param	targets_ptr_	Pointer to the targets interface
\param	select_mode_	Target selection mode: one of TargetSelectionMode
\param  num_of_uavs_	Number of UAVs
\param  alpha_		Distance Weight (coefficient) for WEIGHTED_SCORE_AND_DISTANCE target selection mode
**/
TaskAllocator::TaskAllocator(CentralizedEstimator* targets_estimation_ptr_, TargetSelectionMode mode_, int num_of_uavs_, double alpha_)
{
	num_of_uavs = num_of_uavs_;

	// Assign targets pointer
	targets_estimation_ptr = targets_estimation_ptr_;
	
	// Resize vector for three UAVs
	uav.resize(num_of_uavs);
	
	// Init UAVs position
	for(int i=0; i<num_of_uavs; i++)
	{
		uav[i].id = i+1;
		uav[i].x = 0.0;
		uav[i].y = 0.0;
		uav[i].z = 0.0;
	}
	
	// Select mode
	mode = mode_;
	
	// Target selection coefficient
	alpha = alpha_;
	
	// Pre-compute maximum distance in the Arena [m]
	maximum_distance = sqrtf(LONG_ARENA*LONG_ARENA + ALT_ARENA*ALT_ARENA);
}

/// Destructor
TaskAllocator::~TaskAllocator()
{
	// Clear and free the vectors
	uav.clear();	
	std::vector<Uav>().swap(uav);
}

/** Update a UAV position given its identifier
\param	id		UAV identifier
\params	x,y,z		UAV position
**/
void TaskAllocator::updateUavPosition(int id, double x, double y, double z)
{
	if(id < 1 && id > num_of_uavs )
	{
		ROS_ERROR("TaskAllocator::updateUavPosition() id argument must be bigger than 1 and lower than %d", num_of_uavs);
		exit(0);
	}
	
	uav[id-1].id = id;
	uav[id-1].x = x;
	uav[id-1].y = y;
	uav[id-1].z = z;
}

/** Get optimal Target according to the selected mode. If all targets has 
    UNKNOWN score (i.e., color) the optimal target 
    will be the nearest to the UAV.
\param	id		UAV identifier
\return	target_id	Optimal target identifier (-1 if it does not exist)
**/
int TaskAllocator::getOptimalTarget(int id)
{	
	// Get valid targets info --> targets vector
	std::vector<Target> targets;
	Target tmp_target;
	Color target_color;

	std::vector<int> active_targets = targets_estimation_ptr->getActiveTargets();

	for(int i = 0; i < active_targets.size(); i++)
	{
		tmp_target.id = active_targets[i];
		targets_estimation_ptr->getTargetInfo(active_targets[i], tmp_target.x, tmp_target.y, tmp_target.status, target_color);

		if(tmp_target.status == UNASSIGNED)
		{
			switch(target_color)
			{
				case UNKNOWN: 
				tmp_target.score = UNKNOWN_S;
				tmp_target.priority = LOW_PRIORITY;
				break;
				case RED:
				tmp_target.score = RED_S;
				tmp_target.priority = STATIC_PRIORITY + tmp_target.score;
				break;
				case GREEN:
				tmp_target.score = GREEN_S;
				tmp_target.priority = STATIC_PRIORITY + tmp_target.score;
				break;
				case BLUE:
				tmp_target.score = BLUE_S;
				tmp_target.priority = STATIC_PRIORITY + tmp_target.score;
				break;
				case YELLOW:
				tmp_target.score = YELLOW_S;
				tmp_target.priority = MOVING_PRIORITY;
				break;
				case ORANGE:
				tmp_target.score = YELLOW_S;
				tmp_target.priority = BIG_PRIORITY;
				break;
			}
			targets.push_back(tmp_target);
		}
	}
	
	// Get optimal target: lower difficulty and/or nearest target in 'targets', according to selected mode
	int optimal_target_id = -1; 					// Optimal target result
	double max_priority = getMaxPriority(targets);
	double minimum_distance = std::numeric_limits<double>::max();	// Minimum distance in all or a group of estimated targets
	double distance = std::numeric_limits<double>::max(); 		// Distance from UAV to targets
	double norm_distance = std::numeric_limits<double>::max();	// Normalized distance from UAV to targets according to the Arena field dimensions
	double norm_score = ORANGE_DOUBLE_S;					// Normalized score according to the maximum difficult
	double J = std::numeric_limits<double>::max();			// Ratio between distance and difficult
	double Jmin = std::numeric_limits<double>::max();		// Minimum ratio between distance and difficult
	
	TargetSelectionMode tmp_mode = mode;
	
	for(int i = 0; i < targets.size(); i++)
	{
		switch(mode)
		{
			case NEAREST:
				
				// Only by distance (lower)
				distance = getModule(targets[i].x - uav[id-1].x, targets[i].y - uav[id-1].y);
				
				if(distance < minimum_distance)
				{
					minimum_distance = distance;
					optimal_target_id = targets[i].id;
				}
				break;
			
			case HIGHER_PRIORITY_NEAREST:
				
				// By priority (higher) and after distance (lower)
				
				if(targets[i].priority == max_priority) 
				{
					distance = getModule(targets[i].x - uav[id-1].x, targets[i].y - uav[id-1].y);
					if(distance < minimum_distance)
					{
						minimum_distance = distance;
						optimal_target_id = targets[i].id;
					}
				}
				
				break;
			
			case WEIGHTED_SCORE_AND_DISTANCE:
				
				// Weighted Selection
				norm_distance = getModule(targets[i].x - uav[id-1].x, targets[i].y - uav[id-1].y)/maximum_distance;
				norm_score = (double)targets[i].score/(double)ORANGE_DOUBLE_S;
				
				J = alpha * norm_distance - (1.0-alpha) * norm_score;
				
				if(J < Jmin)
				{
					Jmin = J;
					optimal_target_id = targets[i].id;
				}
				
				break;
			
			default:
				ROS_ERROR("TaskAllocator::TaskAllocator() mode argument must be one of enumered by TargetSelectionMode");
				exit(0);
		}
	}
	
	// Clean vars
	targets.clear();
	std::vector<Target>().swap(targets);
	
	
	return optimal_target_id;
}



// Auxiliar function that returns the module of [dx,dy]
double TaskAllocator::getModule(double dx, double dy)
{
	return sqrtf(dx*dx+dy*dy);
}

// Auxiliar function that returns the maximum priority inside the 'targets' vect
double TaskAllocator::getMaxPriority(std::vector<Target> targets_)
{
	double max_priority = LOW_PRIORITY; // minimum 

	for(int i = 0; i < targets_.size(); i++)
	{
		if(targets_[i].priority > max_priority)
		{
			max_priority = targets_[i].priority;
		}
	}
	
	return max_priority;
}

}
