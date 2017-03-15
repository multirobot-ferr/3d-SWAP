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

#ifndef TARGET_TRACKER_H_
#define TARGET_TRACKER_H_

#include <mbzirc_scheduler/Candidate.h>
#include <mbzirc_scheduler/timer.hpp>

#include <vector>
#include <Eigen/Eigen>


namespace mbzirc {

enum TargetStatus {UNASSIGNED, ASSIGNED, CAUGHT, DEPLOYED, LOST, FAILED, N_STATUS};
enum Color {UNKNOWN = -1, RED = 0, BLUE, GREEN, YELLOW, ORANGE, N_COLORS};
enum Factor {COLOR};
    
/** \brief This class implements a stochastic filter for an object. 

This class implements a stochastic filter for an object. The filter estimates some continuous features 
(e.g., the position and velocity) and some discrete features (e.g., shape or color). The object may be static or 
moving. 

*/

class TargetTracker 
{
public:
	TargetTracker(int id);
	~TargetTracker();

	void initialize(Candidate* z);
	void predict(double dt);
	bool update(Candidate* z);
	double getLikelihood(Candidate* z);
	double getDistance(Candidate* z);
	double lastUpdateTime();
	int getUpdateCount();
	void getPose(double &x, double &y);
	void getVelocity(double &vx, double &vy);
	std::vector<std::vector<double> > getCov();
	int getNumFactors();
	std::vector<double> getFactorProbs(int factor);

	TargetStatus getStatus();
	void setStatus(TargetStatus status);
	int getId();
	bool isLarge();
	bool isStatic();
	Color getColor();

protected:
	Timer update_timer_;			/// Timer for last update
	int update_count_;				/// Counter with the number of updates
	int id_;						/// Target identifier
	bool is_static_;				/// It indicates whether the target is static/dynamic
	bool is_large_;					/// It indicates whether the targes is large
	TargetStatus status_;			/// Current status

	/// Factored discrete belief: COLOR	
	std::vector<std::vector<double> > fact_bel_;	
	
	/// State vector: [x (m), y (m), vx (m/s), vy (m/s)]
	Eigen::MatrixXd pose_;
	Eigen::MatrixXd pose_cov_;

};

}

#endif
