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

// #define DEBUG_MODE

using namespace std;

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
\param z_list List with observations to update
\return True if everything was fine
*/
bool CentralizedEstimator::update(vector<Candidate*> z_list)
{
	vector<vector<double> > distances;
	vector<int> valid_targets;
	vector<int> valid_candidates;
	int n_valid_targets = 0, n_valid_candidates = 0;

	// Compute distances for each association. And count valid targets
	for(auto it = targets_.begin(); it != targets_.end(); ++it)
	{
		bool target_valid = true;

		if((it->second)->getStatus() != CAUGHT && (it->second)->getStatus() != DEPLOYED && (it->second)->getStatus() != LOST)
		{
			valid_targets.push_back((it->second)->getId());
			n_valid_targets++;
		}
		else
		{
			valid_targets.push_back(-1);
			target_valid = false;
		}

		vector<double> t_distances;
		double likelihood;

		for(int i = 0; i < z_list.size(); i++)
		{
			if(target_valid)
			{		
				likelihood = (it->second)->getLikelihood(z_list[i]);
				t_distances.push_back(likelihood);
			}
			else
				t_distances.push_back(-1.0);
		}

		distances.push_back(t_distances);
	}

	// All candidates valid initially
	for(int i = 0; i < z_list.size(); i++)
		valid_candidates.push_back(1);

	n_valid_candidates = z_list.size();

	#ifdef DEBUG_MODE
	cout << "Candidates from UAV " << endl;
	cout << "Distances: " << endl;
	for(int i = 0; i < distances.size(); i++)
	{
		for(int j = 0; j < distances[i].size(); j++)
			cout << distances[i][j] << " ";
		cout << endl;
	}
	cout << endl;	
	
	#endif

	// Look for best pairs until running out of candidates or targets
	while(n_valid_targets != 0 && n_valid_candidates != 0 )
	{
		double min_dist = -1.0;
		pair<int, int> best_pair;

		for(int t_id = 0; t_id < distances.size(); t_id++)
		{
			if(valid_targets[t_id] != -1)
			{
				for(int c_id = 0; c_id < distances[t_id].size(); c_id++)
				{
					if(valid_candidates[c_id] != -1 && (min_dist == -1.0 || distances[t_id][c_id] < min_dist))
					{
						min_dist = distances[t_id][c_id];
						best_pair.first = t_id;
						best_pair.second = c_id;
					}		
				}
			}
		}

		// If there is no good data association, create new target
		if(min_dist <= likelihood_th_)
		{
			#ifdef DEBUG_MODE
			cout << "Candidate " << z_list[best_pair.second]->location(0) << "," << z_list[best_pair.second]->location(1) << ". Associated to target " << valid_targets[best_pair.first] << ", with distance " << min_dist << endl;
			#endif

			targets_[valid_targets[best_pair.first]]->update(z_list[best_pair.second]);
		}
		else
		{
			#ifdef DEBUG_MODE
			cout << "Candidate " << z_list[best_pair.second]->location(0) << "," << z_list[best_pair.second]->location(1) << ". New target " << track_id_count_ << ", with distance " << min_dist << endl;
			#endif
			int new_target_id = track_id_count_++;
			targets_[new_target_id] = new TargetTracker(new_target_id);
			targets_[new_target_id]->initialize(z_list[best_pair.second]);

			// Include new target's distances
			valid_targets.push_back(new_target_id);
			n_valid_targets++;

			vector<double> t_distances;
			double likelihood;

			for(int i = 0; i < z_list.size(); i++)
			{
				if(valid_candidates[i] != -1)
				{		
					likelihood = targets_[new_target_id]->getLikelihood(z_list[i]);
					t_distances.push_back(likelihood);
				}
				else
					t_distances.push_back(-1.0);
			}

			distances.push_back(t_distances);			
		}

		// Update with best pair and remove it
		//valid_targets[best_pair.first] = -1;
		//n_valid_targets--;

		valid_candidates[best_pair.second] = -1;
		n_valid_candidates--;
	}

	// Create new targets with remaining candidates
	if(n_valid_candidates)
	{
		for(int i = 0; i < z_list.size(); i++)
		{
			if(valid_candidates[i])
			{
				#ifdef DEBUG_MODE
				cout << "Candidate " << z_list[i]->location(0) << "," << z_list[i]->location(1) << ". New target " << track_id_count_ << endl;
				#endif

				targets_[track_id_count_] = new TargetTracker(track_id_count_);
				targets_[track_id_count_++]->initialize(z_list[i]);
			}
		}
	}
}

/// Return number of targets
int CentralizedEstimator::getNumTargets()
{
	return targets_.size();
}

/// Return Identifiers of active targets not caught, lost or deployed
vector<int> CentralizedEstimator::getActiveTargets()
{
	vector<int> targets_ids;

	for(auto it = targets_.begin(); it != targets_.end(); ++it)
	{
		if( (it->second)->getStatus() != LOST && (it->second)->getStatus() != CAUGHT && (it->second)->getStatus() != DEPLOYED )
		{
			targets_ids.push_back((it->second)->getId());
		}
	}

	return targets_ids;
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

/** \brief Return position information from a target
\param target_id Identifier of the target
\param x Position of the target
\param y Position of the target
\param covariance Covariance matrix for position
\return True if the target was found 
*/
bool CentralizedEstimator::getTargetInfo(int target_id, double &x, double &y, vector<vector<double> > &covariances)
{
	bool found = false;

	auto it = targets_.find(target_id);
	if(it != targets_.end())
	{
		found = true;
		targets_[target_id]->getPose(x, y);
		covariances = targets_[target_id]->getCov();
	}
	
	return found;
}

/** \brief Return position and velocity information from a target
\param target_id Identifier of the target
\param x Position of the target
\param y Position of the target
\param covariance Covariance matrix for position
\param vx Velocity of the target
\param vy Velocity of the target
\return True if the target was found 
*/
bool CentralizedEstimator::getTargetInfo(int target_id, double &x, double &y, vector<vector<double> > &covariances, double &vx, double &vy)
{
	bool found = false;

	auto it = targets_.find(target_id);
	if(it != targets_.end())
	{
		found = true;
		targets_[target_id]->getPose(x, y);
		covariances = targets_[target_id]->getCov();
		targets_[target_id]->getVelocity(vx, vy);
	}
	
	return found;
}

/** \brief Set the status of a target
\param target_id Identifier of the target
\param Status Status of the target 
\return True if the target was found 
*/
bool CentralizedEstimator::setTargetStatus(int target_id, TargetStatus status)
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
		if( ((it->second)->getStatus() == LOST) || ((it->second)->isStatic() == false && (it->second)->lastUpdateTime() > lost_th_) )
		{
			delete(it->second);
			it = targets_.erase(it);
		}
		else
			++it;
	}
}

/** Print information from the targets for debugging
*/
void CentralizedEstimator::printTargetsInfo()
{
	cout << "****************************************************" << endl;
	cout << "Number of targets: " << targets_.size() << endl;

	for(auto it = targets_.begin(); it != targets_.end(); ++it)
	{
		cout << "Id: " << (it->second)->getId() << ". ";

		switch((it->second)->getStatus())
		{
			case UNASSIGNED:
			cout << "Status: " << "UNASSIGNED. "; 
			break;
			case ASSIGNED:
			cout << "Status: " << "ASSIGNED. "; 
			break;
			case CAUGHT:
			cout << "Status: " << "CAUGHT. "; 
			break;
			case DEPLOYED:
			cout << "Status: " << "DEPLOYED. "; 
			break;
			case LOST:
			cout << "Status: " << "LOST. "; 
			break;
			default:
			cout << "Status: " << "ERROR. ";
		}

		if( (it->second)->getStatus() != LOST && (it->second)->getStatus() != CAUGHT && (it->second)->getStatus() != DEPLOYED )
		{
			double x, y, vx, vy;
			vector<vector<double> > cov;
			vector<double> color_probs;

			(it->second)->getPose(x,y);
			(it->second)->getVelocity(vx,vy);
			cov = (it->second)->getCov();
			color_probs = (it->second)->getFactorProbs(0);

			cout << "Position: " << x << "," << y << ". Velocity: " << vx << "," << vy << ". Covariances: " << cov[0][0] << " " << cov[0][1] << "; " << cov[1][0] << " " << cov[1][1] << "." << endl;
			cout << "Color: ";
			switch((it->second)->getColor())
			{
				case UNKNOWN:
				cout << "UNKNOWN. ";
				break;
				case RED:
				cout << "RED. ";
				break;
				case BLUE:
				cout << "BLUE. ";
				break;
				case GREEN:
				cout << "GREEN. ";
				break;
				case YELLOW:
				cout << "YELLOW. ";
				break;
				case ORANGE:
				cout << "ORANGE. ";
				break;
				default:
				cout << "ERROR. ";
			}

			cout << "( ";
			for(int i = 0; i < color_probs.size(); i++)
				cout << color_probs[i] << " ";

			cout << "). ";

			cout << "Static? ";
			if((it->second)->isStatic())
				cout << "yes. ";
			else
				cout << "no. ";

			cout << "Large? ";
			if((it->second)->isLarge())
				cout << "yes. ";
			else
				cout << "no. ";
		}
		cout << endl;
	}
}

}
