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
**/
TaskAllocator::TaskAllocator(CentralizedEstimator* targets_estimation_ptr_, TargetSelectionMode mode_, int num_of_uavs_)
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
    UNKNOWN difficulty (in other word: score/color) the optimal target 
    will be the nearest to the UAV.
\param	id		UAV identifier
\return	target_id	Optimal target identifier (-1 if it doesn not exist)
**/
int TaskAllocator::getOptimalTarget(int id)
{	
	// Get valid targets info
	int targets_num = targets_estimation_ptr->getNumTargets();
	std::vector<Target> targets;
	Target tmp_target;
	for(int i=0; i<targets_num; i++)
	{
		tmp_target.id = i;
		targets_estimation_ptr->getTargetInfo(i, tmp_target.x, tmp_target.y, tmp_target.status, tmp_target.score);
		if(tmp_target.status == UNASSIGNED)
			targets.push_back(tmp_target);
	}
	
	// Get optimal target: lower difficulty and/or nearest target in 'targets', according to selected mode
	int optimal_target_id = -1; 
	double minimum_distance = std::numeric_limits<double>::max();
	Color minimum_difficult = getMinScore(targets);
	double mod; // distance from UAV to targets
	
	TargetSelectionMode tmp_mode = mode;
	if(minimum_difficult == UNKNOWN)	// if all targets difficult are unknown, set NEAREST mode by default
		tmp_mode = NEAREST;
	
	for(int i=0; i<targets.size(); i++)
	{
		switch(mode)
		{
			case NEAREST:
				
				// Only by distance (lower)
				mod = getModule(targets[i].x - uav[id].x, targets[i].y - uav[id].y);
				if(mod < minimum_distance)
				{
					minimum_distance = mod;
					optimal_target_id = targets[i].id;
				}
				break;
			
			case LOWER_SCORE_NEAREST:
				
				// By difficulty (lower) and after distance (lower)
				if(targets[i].score == minimum_difficult)
				{
					mod = getModule(targets[i].x - uav[id].x, targets[i].y - uav[id].y);
					if(mod < minimum_distance)
					{
						minimum_distance = mod;
						optimal_target_id = targets[i].id;
					}
				}
				
				break;
			
			case WEIGHTED_SCORE_AND_DISTANCE:
				
				// Weighted Selection
				/// ****************************** todo *****************************************
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

// Auxiliar function that returns the minimum score inside the 'targets' vect
Color TaskAllocator::getMinScore(std::vector<Target> targets_)
{
	Color min_score = ORANGE; //maximum (4)
	bool one_known_at_least = false;
	for(int i=0; i<targets_.size(); i++)
	{
		if(targets_[i].score != UNKNOWN)
		{
			one_known_at_least = true;
			if(targets_[i].score < min_score)
				min_score = targets_[i].score;
		}
		
	}
	
	if(one_known_at_least)
		return min_score;
	else
		return UNKNOWN;
}


}



