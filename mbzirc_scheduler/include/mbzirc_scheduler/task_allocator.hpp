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
#include <mbzirc_scheduler/centralized_estimator.h>

#ifndef TASK_ALLOCATOR_HPP_
#define TASK_ALLOCATOR_HPP_

// Arena dimmensions
#define LONG_ARENA	90.0
#define ALT_ARENA	60.0


enum TargetSelectionMode {NEAREST = 1, LOWER_SCORE_NEAREST = 2, WEIGHTED_SCORE_AND_DISTANCE = 3};
// NEAREST: get the closest target to the UAV
// LOWER_SCORE_NEAREST: get the easier target closest to the UAV
// WEIGHTED_SCORE_AND_DISTANCE: TODO

using namespace std;

namespace mbzirc {

// Simple UAV data info
class Uav
{
	public:
	   int id;		// Unique identifier
	   double x,y,z;	// Global Position

};

class Target
{
	public:
	   int id;		// Unique identifier
	   TargetStatus status;	// Possible status: {UNASSIGNED, ASSIGNED, CAUGHT, DEPLOYED, LOST, N_STATUS}
	   Color score;		// Score/Difficulty according to the color: {UNKNOWN = -1, RED = 0, GREEN, BLUE, YELLOW, ORANGE, N_COLORS}
	   double x,y;		// Global Position
};

bool getTargetInfo(int target_id, double &x, double &y, TargetStatus &type, Color &color);

// Task allocator: class to get the optimal target to a UAV given the current targets estimations and the selection mode
class TaskAllocator 
{
	public:
		/**
		 * Constructor
		**/
		TaskAllocator(CentralizedEstimator* targets_estimation_ptr_, TargetSelectionMode mode_, int num_of_uavs_, double alpha_);
		
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
	
		// Return module of [dx,dy]
		double getModule(double dx, double dy);
		
		// Return the minimum score inside the 'targets' vect
		Color getMinScore(std::vector<Target> targets_);
	
		// Pointer to the targets interface
		CentralizedEstimator* targets_estimation_ptr;
		
		// UAVs positions
		std::vector<Uav> uav;
		
		// Number of UAVs in operation
		int num_of_uavs;		
		
		// Target selection mode
		TargetSelectionMode mode;
		
		// Distance Weight for WEIGHTED_SCORE_AND_DISTANCE target selection mode
		double alpha;
		
		// Diagonal of Arena field [m]
		double maximum_distance;
};

}

#endif
