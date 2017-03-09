//------------------------------------------------------------------------------
// GRVC MBZIRC
// Author Jesus Capitan <jcapitan@us.es>
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

#ifndef CENTRALIZED_ESTIMATOR_H_
#define CENTRALIZED_ESTIMATOR_H_

#include <map>
#include <vector>
#include <iostream>

#include <mbzirc_scheduler/target_tracker.h>

namespace mbzirc {

/** \brief This class implements a centralized filter for all the objects in MBZIRC challenge.

This class implements a centralized filter for all the objects in MBZIRC challenge. A table with all objects is maintained, and a filter for each object, static or moving. 

*/
class CentralizedEstimator
{
public:

	CentralizedEstimator(double lkhd_th, double lost_th);
	~CentralizedEstimator();

	void predict(double dt);
	bool update(std::vector<Candidate*> z_list);

	int getNumTargets();
	std::vector<int> getActiveTargets();
	bool getTargetInfo(int target_id, double &x, double &y, TargetStatus &type, Color &color);
	bool getTargetInfo(int target_id, double &x, double &y, std::vector<std::vector<double> > &covariances);
	bool getTargetInfo(int target_id, double &x, double &y, std::vector<std::vector<double> > &covariances, double &vx, double &vy);
	bool setTargetStatus(int target_id, TargetStatus status);
	void removeLostTargets();
	void printTargetsInfo();

protected:

	std::map<int, TargetTracker *> targets_;	/// Map with targets
	double likelihood_th_;				/// Minimum likelihood threshold for data association
	double lost_th_;				/// Maximum time threshold to lose target
	int track_id_count_;				/// Counter of tracks identifiers
};

}

#endif
