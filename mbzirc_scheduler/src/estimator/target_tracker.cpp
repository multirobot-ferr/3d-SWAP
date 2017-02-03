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

namespace mbzirc {

/** Constructor
\param id Identifier
*/
TargetTracker::TargetTracker(int id)
{
	id_ = id;

	fact_bel_.resize(3);
	fact_bel_[STATUS].resize(N_STATUS);
	fact_bel_[COLOR].resize(N_COLORS);
	fact_bel_[SHAPE].resize(N_SHAPES);

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
void TargetTracker::initialize(Candidate z)
{
	int i;

	// Init factored belief
	for(int fact = 0; fact < fact_bel_.size(); fact++)
	{
		switch(fact)
		{
			case STATUS:

			for(i = 0; i < N_STATUS; i++)
			{
				if(i == UNKNOWN)
					fact_bel_[STATUS][i] = 1.0;
				else
					fact_bel_[STATUS][i] = 0.0;
			}	
			break;

			case COLOR:

			for(i = 0; i < N_COLORS; i++)
			{
				fact_bel_[COLOR][i] = 1.0/N_COLORS;
			}	
			break;
			
			case SHAPE:

			for(i = 0; i < N_SHAPES; i++)
			{
				fact_bel_[STATUS][i] = 1.0/N_SHAPES;
			}	
			break;
		}
	}

	// Setup state vector
	pose_.setZero(4, 1);
	pose_(0,0) = z.location(0);
	pose_(1,0) = z.location(1);
	pose_(2,0) = z.speed(0);
	pose_(3,0) = z.speed(1);
		
	// Setup cov matrix
	pose_cov_.setIdentity(4, 4);
	pose_cov_(0,0) = z.locationCovariance(0,0);
	pose_cov_(0,1) = z.locationCovariance(0,1);
	pose_cov_(1,0) = z.locationCovariance(1,0);
	pose_cov_(1,1) = z.locationCovariance(1,1);

	pose_cov_(2,2) = z.speedCovariance(0,0);
	pose_cov_(2,3) = z.speedCovariance(0,1);
	pose_cov_(3,2) = z.speedCovariance(1,0);
	pose_cov_(3,3) = z.speedCovariance(1,1);

	// Update timer
	update_timer_.reset();
}

/**
\brief Predict the filter.
\param dt Length in seconds of the prediction step. 
*/
void TargetTracker::predict(double dt)
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

/**
\brief Update the filter.
\param z Observation to update. 
\return True if everything was fine
*/
bool TargetTracker::update(Candidate z)
{
	// Compute update jacobian
	Eigen::Matrix<double, 2, 4> H;
	H.setZero(2, 4);
	H(0,0) = 1.0;
	H(1,1) = 1.0;
		
	// Compute update noise matrix
	Eigen::Matrix<double, 2, 2> R;
	R(0,0) = z.locationCovariance(0,0);
	R(0,1) = z.locationCovariance(0,1);
	R(1,0) = z.locationCovariance(1,0);
	R(1,1) = z.locationCovariance(1,1);
		
	// Calculate innovation matrix
	Eigen::Matrix<double, 2, 2> S;
	S = H*pose_cov_*H.transpose() + R;
		
	// Calculate kalman gain
	Eigen::Matrix<double, 4, 2> K;
	K = pose_cov_*H.transpose()*S.inverse();
		
	// Calculate innovation vector
	Eigen::Matrix<double, 2, 1> y;
	y(0,0) = z.location(0) - pose_(0,0);
	y(1,0) = z.location(1) - pose_(1,0);
		
	// Calculate new state vector
	pose_ = pose_ + K*y;
		
	// Calculate new cov matrix
	Eigen::Matrix<double, 4, 4> I;
	I.setIdentity(4, 4);
	pose_cov_ = (I - K*H)*pose_cov_;

	// Update timer
	update_timer_.reset();
}
    
/**
Compute the likelihood of an observation with current belief.
\param z Observation. 
\return Likelihood measurement
*/
double TargetTracker::getLikelihood(Candidate z)
{
}

/**
Return the time since the last observation update. 
\return Update time
*/
double TargetTracker::lastUpdateTime()
{
	return update_timer_.elapsed();
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

/** \brief Return likeliest target status
\return Target status  
*/
TargetStatus TargetTracker::getStatus()
{
	TargetStatus status;
	double max_prob = -1.0;

	for(int i = 0; i < N_STATUS; i++)
	{
		if(fact_bel_[STATUS][i] > max_prob)
		{
			max_prob = fact_bel_[STATUS][i];
			status = (TargetStatus)i;
		}
	}

	return status;
}

/** \brief Set a new target status
\param Target status  
*/
void TargetTracker::setStatus(TargetStatus status)
{
	for(int i = 0; i < N_STATUS; i++)
	{
		if(i == status)
			fact_bel_[STATUS][i] = 1.0;
		else
			fact_bel_[STATUS][i] = 0.0;
	}		
}

/** \brief Return target identifier
\return Target identifier  
*/
int TargetTracker::getId()
{
	return id_;
}

}

