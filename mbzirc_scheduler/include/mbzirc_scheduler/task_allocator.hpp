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

#include <ros/ros.h>
#include <mbzirc_scheduler/centralized_estimator.h>

#ifndef TASK_ALLOCATOR_HPP_
#define TASK_ALLOCATOR_HPP_

#define NUM_OF_UAVS 3	// Number of UAVs in operation

using namespace std;

namespace mbzirc {

// Simple UAV data info
class Uav
{
	public:
	   int id;		// Unique identifier
	   double x,y,z;	// Global Position

};

// Task allocator: class to get the optimal target to a UAV given the current targets estimations
class TaskAllocator 
{
	public:
		/**
		 * Constructor
		**/
		TaskAllocator(CentralizedEstimator* targets_ptr_);
		
		/**
		 * Destructor
		**/
		~TaskAllocator();
		
		/**
		 * Update a UAV position given its identifier
		**/
		void updateUavPosition(int id, double x, double y, double z);
		
		/**
		 * Get optimal Target given the UAV identifier (-1 if error)
		**/
		int getOptimalTarget(int id);
		
	protected:
	
		// Pointer to the targets interface
		CentralizedEstimator* targets_ptr;
		
		// UAVs positions
		std::vector<Uav> uav;
};

}

#endif
