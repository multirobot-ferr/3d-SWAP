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


#include <mbzirc_scheduler/target_tracker.h>

#define VEL_NOISE_VAR 0.2 
#define COLOR_DETECTOR_PD 0.9
#define MIN_COLOR_DISTANCE 0.15

using namespace std;

namespace mbzirc {

/** Constructor
\param id Identifier
*/
TargetTracker::TargetTracker(int id)
{
	id_ = id;

	is_static_ = false;
	is_large_ = false;
	status_ = UNASSIGNED;

	fact_bel_.resize(1);
	fact_bel_[COLOR].resize(N_COLORS);

	pose_ = Eigen::MatrixXd::Zero(4,1);
	pose_cov_ = Eigen::MatrixXd::Identity(4,4);
}

/// Destructor
TargetTracker::~TargetTracker()
{
}

/**
\brief Initialize the filter. 
\param z Initial observation
*/
void TargetTracker::initialize(Candidate* z)
{

	// Setup state vector
	pose_.setZero(4, 1);
	pose_(0,0) = z->location(0);
	pose_(1,0) = z->location(1);
	pose_(2,0) = 0.0;
	pose_(3,0) = 0.0;
		
	// Setup cov matrix
	pose_cov_.setIdentity(4, 4);
	pose_cov_(0,0) = 2*z->locationCovariance(0,0);
	pose_cov_(0,1) = 2*z->locationCovariance(0,1);
	pose_cov_(1,0) = 2*z->locationCovariance(1,0);
	pose_cov_(1,1) = 2*z->locationCovariance(1,1);

	pose_cov_(2,2) = VEL_NOISE_VAR;
	pose_cov_(3,3) = VEL_NOISE_VAR;


	// Init and update factored belief 
	for(int fact = 0; fact < fact_bel_.size(); fact++)
	{
		switch(fact)
		{
			case COLOR:

			double prob_z, total_prob = 0.0;

			for(int i = 0; i < N_COLORS; i++)
			{
				if(z->color == i)
					prob_z = COLOR_DETECTOR_PD;
				else
					prob_z = (1.0 - COLOR_DETECTOR_PD)/(N_COLORS-1);

				fact_bel_[COLOR][i] = (1.0/(N_COLORS))*prob_z;
				total_prob += fact_bel_[COLOR][i];
			}
			
			// Normalize
			for(int i = 0; i < N_COLORS; i++)
			{
				fact_bel_[COLOR][i] /= total_prob;
			}	

			break;			
		}
	}

	if(fact_bel_[COLOR][ORANGE] > (1.0 - fact_bel_[COLOR][ORANGE]))
		is_large_ = true;
	else
		is_large_ = false;
 
	if(fact_bel_[COLOR][YELLOW] > (1.0 - fact_bel_[COLOR][YELLOW]))
	{
		is_static_ = false;
	}
	else
	{
		is_static_ = true;
	}

	// Update timer
	update_timer_.reset();
	update_count_ = 0;
}

/**
\brief Predict the filter.
\param dt Length in seconds of the prediction step. 
*/
void TargetTracker::predict(double dt)
{
	// static factors do not vary. Position depending on whether it is dynamic or not.

	if(!is_static_)
	{
		// State vector prediction
		pose_(0,0) += pose_(2,0)*dt;
		pose_(1,0) += pose_(3,0)*dt;
		
		// Convariance matrix prediction
		Eigen::Matrix<double, 4, 4> F;
		F.setIdentity(4, 4);
		F(0,2) = dt;
		F(1,3) = dt;
		Eigen::Matrix<double, 4, 4> Q;
		Q.setZero(4, 4);
		Q(2,2) = VEL_NOISE_VAR*dt*dt;
		Q(3,3) = VEL_NOISE_VAR*dt*dt;
		pose_cov_ = F*pose_cov_*F.transpose() + Q;
	}
}

/**
\brief Update the filter.
\param z Observation to update. 
\return True if everything was fine
*/
bool TargetTracker::update(Candidate* z)
{
	// Update factored belief 
	for(int fact = 0; fact < fact_bel_.size(); fact++)
	{
		switch(fact)
		{
			case COLOR:

			double prob_z, total_prob = 0.0;

			for(int i = 0; i < N_COLORS; i++)
			{
				if(z->color == i)
					prob_z = COLOR_DETECTOR_PD;
				else
					prob_z = (1.0 - COLOR_DETECTOR_PD)/(N_COLORS-1);

				fact_bel_[COLOR][i] *= prob_z;
				total_prob += fact_bel_[COLOR][i];
			}
			
			// Normalize
			for(int i = 0; i < N_COLORS; i++)
			{
				fact_bel_[COLOR][i] /= total_prob;
			}	

			break;			
		}
	}

	if(fact_bel_[COLOR][ORANGE] > (1.0 - fact_bel_[COLOR][ORANGE]))
		is_large_ = true;
	else
		is_large_ = false;
 
	if(fact_bel_[COLOR][YELLOW] > (1.0 - fact_bel_[COLOR][YELLOW]))
	{
		is_static_ = false;
	}
	else
	{
		is_static_ = true;
	}
	
	// Compute update jacobian
	Eigen::Matrix<double, 2, 4> H;
	H.setZero(2, 4);
	H(0,0) = 1.0;
	H(1,1) = 1.0;
		
	// Compute update noise matrix
	Eigen::Matrix<double, 2, 2> R;
	R(0,0) = z->locationCovariance(0,0);
	R(0,1) = z->locationCovariance(0,1);
	R(1,0) = z->locationCovariance(1,0);
	R(1,1) = z->locationCovariance(1,1);
		
	// Calculate innovation matrix
	Eigen::Matrix<double, 2, 2> S;
	S = H*pose_cov_*H.transpose() + R;
		
	// Calculate kalman gain
	Eigen::Matrix<double, 4, 2> K;
	K = pose_cov_*H.transpose()*S.inverse();
		
	// Calculate innovation vector
	Eigen::Matrix<double, 2, 1> y;
	y = H*pose_;
	y(0,0) = z->location(0) - y(0,0);
	y(1,0) = z->location(1) - y(1,0);
		
	// Calculate new state vector
	pose_ = pose_ + K*y;
		
	// Calculate new cov matrix
	Eigen::Matrix<double, 4, 4> I;
	I.setIdentity(4, 4);
	pose_cov_ = (I - K*H)*pose_cov_;

	// Update timer
	update_timer_.reset();
	update_count_++;
}
    
/**
Compute the likelihood of an observation with current belief. Based on Mahalanobis distance. 
\param z Observation. 
\return Likelihood measurement
*/
double TargetTracker::getLikelihood(Candidate* z)
{
	double distance;

	// Compute update jacobian
	Eigen::Matrix<double, 2, 4> H;
	H.setZero(2, 4);
	H(0,0) = 1.0;
	H(1,1) = 1.0;
		
	// Compute update noise matrix
	Eigen::Matrix<double, 2, 2> R;
	R(0,0) = z->locationCovariance(0,0);
	R(0,1) = z->locationCovariance(0,1);
	R(1,0) = z->locationCovariance(1,0);
	R(1,1) = z->locationCovariance(1,1);
		
	// Calculate innovation matrix
	Eigen::Matrix<double, 2, 2> S;
	S = H*pose_cov_*H.transpose() + R;
			
	// Calculate innovation vector
	Eigen::Matrix<double, 2, 1> y;
	y = H*pose_;
	y(0,0) = z->location(0) - y(0,0);
	y(1,0) = z->location(1) - y(1,0);

	distance = y.transpose()*S.inverse()*y;
	
	double prob_z, prob_color = 0.0;

	for(int i = 0; i < N_COLORS; i++)
	{
		if(z->color == i)
			prob_z = COLOR_DETECTOR_PD;
		else
			prob_z = (1.0 - COLOR_DETECTOR_PD)/(N_COLORS-1);

		prob_color += fact_bel_[COLOR][i]*prob_z;
	}

	if(prob_color < MIN_COLOR_DISTANCE)
		distance = -1;

	return distance;
}

/**
Compute the euclidean distance of an observation with current belief.
\param z Observation. 
\return Euclidean distance
*/
double TargetTracker::getDistance(Candidate* z)
{
	double dx, dy;
	dx = pose_(0,0) - z->location(0);
	dy = pose_(1,0) - z->location(1);

	return sqrt(dx*dx + dy*dy);
}

/**
Return the time since the last observation update. 
\return Update time
*/
double TargetTracker::lastUpdateTime()
{
	return update_timer_.elapsed();
}

/**
Return the counter of updates. 
\return Update counter
*/
int TargetTracker::getUpdateCount()
{
	return update_count_;
}
    
/** \brief Return pose information from the target
\param x Position of the target
\param y Position of the target
*/
void TargetTracker::getPose(double &x, double &y)
{
	x = pose_(0,0);
	y = pose_(1,0);
}

/** \brief Return velocity information from the target
\param vx Velocity of the target
\param vy Velocity of the target
*/
void TargetTracker::getVelocity(double &vx, double &vy)
{
	vx = pose_(2,0);
	vy = pose_(3,0);
}

/** \brief Return covariance matrix from the target position
\return Covariance matrix
*/
vector<vector<double> > TargetTracker::getCov()
{
	vector<vector<double> > covariance;
	covariance.resize(2);
	covariance[0].resize(2);
	covariance[1].resize(2);

	covariance[0][0] = pose_cov_(0,0);
	covariance[0][1] = pose_cov_(0,1);
	covariance[1][0] = pose_cov_(1,0);
	covariance[1][1] = pose_cov_(1,1);

	return covariance;
}

/** \brief Return number of discrete factors
\return Number of discrete factors
*/
int TargetTracker::getNumFactors()
{
	return fact_bel_.size();
}

/** \brief Return probabilities for a given factor
\param factor Factor to return
\return Probabilities for a given factor
*/
vector<double> TargetTracker::getFactorProbs(int factor)
{
	return fact_bel_[factor];
}

/** \brief Return likeliest target status
\return Target status  
*/
TargetStatus TargetTracker::getStatus()
{
	return status_;
}

/** \brief Set a new target status
\param Target status  
*/
void TargetTracker::setStatus(TargetStatus status)
{
	status_ = status;
}

/** \brief Return target identifier
\return Target identifier  
*/
int TargetTracker::getId()
{
	return id_;
}

/** \brief Return whether target is static or not
\return True if it is static
*/
bool TargetTracker::isStatic()
{
	return is_static_;
}

/** \brief Return whether target is large or not
\return True if it is large
*/
bool TargetTracker::isLarge()
{
	return is_large_;
}

/** \brief Return the likeliest color for the target
\return A color
*/
Color TargetTracker::getColor()
{
	double max_prob = -1.0;
	Color color, color_max;

	for(int i = 0; i < N_COLORS; i++)
	{
		if(fact_bel_[COLOR][i] > max_prob)
		{
			max_prob = fact_bel_[COLOR][i];
			color_max = (Color)i;
		}
	}

	if(max_prob > 1.0 - max_prob)
		color = color_max;
	else
		color = UNKNOWN;

	return color;
}

}

