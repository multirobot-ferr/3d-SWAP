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


#include <mbzirc_scheduler/centralized_estimator.h>

namespace mbzirc {

/** Constructor
\param likelihood_th Likelihood threshold to associate observations
\param lost_th Time threshold to consider target lost
*/
CentralizedEstimator::CentralizedEstimator(double lkhd_th, double lost_th)
{
	likelihood_th_ = lkhd_th;
	lost_th_ = lost_th;
	track_id_count_ = 0;
}

/// Destructor
CentralizedEstimator::~CentralizedEstimator()
{
	for(auto it = targets_.begin(); it != targets_.end(); ++it)
	{
		delete(it->second);
	}
}

/** Prediction step for all targets
\param dt Length in seconds of the prediction step.
*/
void CentralizedEstimator::predict(double dt)
{
	for(auto it = targets_.begin(); it != targets_.end(); ++it)
	{
		if((it->second)->getStatus() != CAUGHT && (it->second)->getStatus() != DEPLOYED)
			(it->second)->predict(dt);
	}
}

/**
\brief Update step for a target
\param z Observation to update
\return True if everything was fine
*/
bool CentralizedEstimator::update(Candidate z)
{
	double max_likelihood = -1, likelihood;
	int best_id;

	for(auto it = targets_.begin(); it != targets_.end(); ++it)
	{
		if((it->second)->getStatus() != CAUGHT && (it->second)->getStatus() != DEPLOYED)
		{
			likelihood = (it->second)->getLikelihood(z);
			if(likelihood > max_likelihood)
			{
				max_likelihood = likelihood;
				best_id = (it->second)->getId();;
			}
		}
	}

	// If there is no good data association, create new target
	if(max_likelihood > likelihood_th_)
		targets_[best_id]->update(z);
	else
	{
		targets_[track_id_count_] = new TargetTracker(track_id_count_);
		targets_[track_id_count_++]->initialize(z);		
	}
}

/// Return number of targets
int CentralizedEstimator::getNumTargets()
{
	return targets_.size();
}

/** \brief Return information from a target
\param target_id Identifier of the target
\param x Position of the target
\param y Position of the target
\param Status Status of the target 
\param color Color of the target
\return True if the target was found 
*/
bool CentralizedEstimator::getTargetInfo(int target_id, double &x, double &y, TargetStatus &status, Color &color)
{
	bool found = false;

	auto it = targets_.find(target_id);
	if(it != targets_.end())
	{
		found = true;
		targets_[target_id]->getPose(x, y);
		status = targets_[target_id]->getStatus();
		color = targets_[target_id]->getColor();
	}
	
	return found;
}

/** \brief Set the status of a target
\param target_id Identifier of the target
\param Status Status of the target 
\return True if the target was found 
*/
bool CentralizedEstimator::setTargetStatus(int target_id, TargetStatus &status)
{
	bool found = false;

	auto it = targets_.find(target_id);
	if(it != targets_.end())
	{
		found = true;
		targets_[target_id]->setStatus(status);
	}
	
	return found;
}

/** Remove lost targets
*/
void CentralizedEstimator::removeLostTargets()
{
	auto it = targets_.begin();

	while(it != targets_.end())
	{		
		if((it->second)->isStatic() == false && (it->second)->lastUpdateTime() > lost_th_)
		{
			delete(it->second);
			it = targets_.erase(it);
		}
		else
			++it;
	}
}

}
