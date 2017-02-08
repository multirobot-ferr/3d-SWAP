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

namespace mbzirc {

/** Constructor
\param	targets_ptr_	Pointer to the targets interface
**/
TaskAllocator::TaskAllocator(CentralizedEstimator* targets_ptr_)
{
	// Assign targets pointer
	targets_ptr = targets_ptr_;
	
	// Resize vector for three UAVs
	uav.resize(NUM_OF_UAVS);
	
	// Init UAVs position
	for(int i=0; i<NUM_OF_UAVS; i++)
	{
		uav[i].id = i+1;
		uav[i].x = 0.0;
		uav[i].y = 0.0;
		uav[i].z = 0.0;
	}
}

/// Destructor
TaskAllocator::~TaskAllocator()
{
	// Clear and free the vectors
	uav.clear();	
	vector<Uav>().swap(uav);
}

/** Update a UAV position given its identifier
\param	id		UAV identifier
\params	x,y,z		UAV position
**/
void TaskAllocator::updateUavPosition(int id, double x, double y, double z)
{
	if(id < 1 && id > NUM_OF_UAVS )
	{
		ROS_ERROR("TaskAllocator::updateUavPosition() id argument must be bigger than 1 and lower than %d", NUM_OF_UAVS);
		exit(0);
	}
	
	uav[id-1].id = id;
	uav[id-1].x = x;
	uav[id-1].y = y;
	uav[id-1].z = z;
}

/** Get optimal Target
\param	id		UAV identifier
\return	target_id	Optimal target identifier (-1 if it doesn not exist)
**/
int TaskAllocator::getOptimalTarget(int id)
{
	int optimal_target = -1;
	
	return optimal_target;
}

}

















