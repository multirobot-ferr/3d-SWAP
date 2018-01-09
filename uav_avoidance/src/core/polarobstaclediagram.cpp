/*
 * Copyright (c) 2017, University of Duisburg-Essen, swap-ferr
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of swap-ferr nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    polarobstaclediagram.cpp
 * @author  Eduardo Ferrera
 * @date    January, 2017
 * @brief   Contains the Polar Obstacle Diagram managers
 *
 * Code of the implementation of the Polar Obstacle Diagram (POD) as a class.
 * The POD tracks all obstacles around the robot that executes swap, and it is used
 * to determine possible conflicts and avoidances direction inside the Swap algorithm
 * The Swap algorithm is an implementation of the following work:
 *
 *         http://dx.doi.org/10.1016/j.robot.2017.01.008
 *
 * @article{ferrera2017decentralized,
 *  title={Decentralized safe conflict resolution for multiple robots in dense scenarios},
 *  author={Ferrera, Eduardo and Capit{\'a}n, Jesus and Casta{\~n}o, Angel R and Marr{\'o}n, Pedro J},
 *  journal={Robotics and Autonomous Systems},
 *  year={2017},
 * publisher={Elsevier}
 * }
 *
 * Please cite.
 *
 */

#include "polarobstaclediagram.h"

namespace avoid
{

    /***************************************************
     *   Methods of the Polar Obstacle Diagram class   *
     * *************************************************/

    /**
     * Initializes the class in an empty way
     */
    PolarObstacleDiagram::PolarObstacleDiagram()
    {
        //ctor
    }

    /* Default destructor */
    PolarObstacleDiagram::~PolarObstacleDiagram()
    {
        //dtor
    }

    /* ****************************************************************************
     *   MAIN METHODS (Intended to control the behaviour of the POD)              *
     * ****************************************************************************/

    /**
     * IsReady returns if the POD is well configurated or not. And if not, explains why.
     */
    bool PolarObstacleDiagram::IsReady(std::string& why_not)
    {
        if ( !id_phi_max_ )
        {
            why_not = "The Polar Obstacle diagram is not ready:\n";
            why_not += "\"granularity\" is not defined. \"granularity\" defines the ammount ";
            why_not += "of directions that will track measurements between -PI and PI.\n";
            why_not += "Use high granularities if you plan to track objects far away from you\n";
            why_not += "Use low granularities if your system has no large computational power\n";
            why_not += "Recomemded value: 360, Min value: 16\n";
            return false;
        }
        if ( !id_r_max_ )
        {
            why_not = "The Polar Obstacle dyagram is not ready:\n";
            why_not += "\"num_measurements\" is zero. \"num_measurements\" defines the ammount ";
            why_not += "of measurements to track on each direction\n";
            why_not += "Use high num_measurements (~5) if your measurements are noisy\n";
            why_not += "Use low num_measurements if your system has no large computational power or almost no noise\n";
            why_not += "Recomemded value: 2, Min value: 1\n";
            return false;
        }
        if ( multi_input_pond_ < 0.0 )
        {
            why_not = "The Polar Obstacle dyagram is not ready:\n";
            why_not += "\"multi_input_pond\" is not defined. \"multi_input_pond\" defines the way that ";
            why_not += "swap ponder the measurements on each direction.\n";
            why_not += "Values close to 1.0 makes ponderation tend to take all measurements in one direction with the same weight\n";
            why_not += "Values close to 0.0 makes the system ponders more values closer to the robot (even if are older values)\n";
            why_not += "The closer to 0.0, the safer, but assumes larger dangerous zones.";
            why_not += "If you have a radio communication system, it is recommended to turn this value to zero.\n";
            why_not += "Eg: RobotX = 0, Object1 = 3, Object2 = 30\n";
            why_not += "A multi_input_pond = 0.1 will create a missmeasurement\n";
            why_not += " d = (1.0 - 0.1/2) * 3.0 + (0.1/2) * 30.0\n";
            why_not += "   =     0.95      * 3.0 +  0.05   * 30.0 = 4.35";
            return false;
        }
        if ( !r_safety_set_ )
        {
            why_not = "The value of the \"Safety Region\" is not set.\n";
            why_not += "The Safety Region is a circular region arround the robot that ";
            why_not += "should be able to overcome his full shape.\n";
            why_not += "Measure your robot and set this parameter accordy.";
            return false;
        }
        if (bracking_distance_ <= 0.0)
        {
            why_not = "The value of the \"Braking Distance\" is not set.\n";
            why_not += "The Braking Distance is the maximum distance required by any robot of the team ";
            why_not += "to fully stop themselves when moving at full speed.\n";
            why_not += "Measure this distance and set this parameter accordy.";
            return false;
        }
        if (local_measurement_error_ < 0.0)
        {
            why_not = "The value of the \"Local Measurement Error\" is not set.\n";
            why_not += "The Local Measurement Error is the maximum expected error in [m] from a local measurement ";
            why_not += "(like the laser ranger placed on the robot)";
            why_not += "Set this parameter according to the specifications of your laser.";
            why_not += "Set it to 0.0 if you do not have local measurements";
            return false;
        }
        if (global_measurement_error_ < 0.0)
        {
            why_not = "The value of the \"Global Measurement Error\" is not set.\n";
            why_not += "The Global Measurement Error is the maximum expected error in [m] from a global measurement ";
            why_not += "(like the the one comming from a shared position in a communication system)";
            why_not += "Set this parameter according to the reliability of the common positionning system.\n";
            why_not += "Set it to 0.0 if you do not have global measurements\n";
            why_not += "The \"Local Measurement Error\" should be set in advance.";
            return false;
        }
        if (gamma_offset_ < 0.0)
        {
            why_not = "The value of the \"Gamma Offset\" is not set.\n";
            why_not += "Gamma offset introduces an offset designated to take into account potential errors, ";
            why_not += "communication delays, etc.";
            why_not += "If your laser is mounted in the top of your robot, and is able only to see the shape of ";
            why_not += "other lasers, this parameter should introduce the difference on the pod.";
            return false;
        }
        if (smooth_factor_ < 0)
        {
            why_not = "The value of the \"Smooth factor\" is not set.\n";
            why_not += "The smooth factor reduces the noise between different measurements of your ranger.\n";
            why_not += "If your laser is not noisy or you have no laser, you can set it to zero and save computational power.\n";
            return false;
        }
        return true;
    }

    /**
     * Forgets the oldest measurement set in order to receive a new one
     */
    void PolarObstacleDiagram::NewMeasurementSetReceived()
    {
        // Older measurements moves to the end of the pod vector.
        std::rotate(dist_.begin(),    dist_.begin()+(id_r_max_-1)*id_phi_max_,    dist_.end());
        std::rotate(zdist_.begin(),   zdist_.begin()+(id_r_max_-1)*id_phi_max_,   zdist_.end());
        std::rotate(dynamic_.begin(), dynamic_.begin()+(id_r_max_-1)*id_phi_max_, dynamic_.end());

        // Old measurements are reset to standard values
        for (unsigned id_phi =0; id_phi < id_phi_max_; ++id_phi)
        {
            dist_[id_phi] = double_inf;
            dynamic_[id_phi] = false;
            zdist_[id_phi]=0;
        }
    }

    /**
     * Sets a measurement value in the POD
     */
    void PolarObstacleDiagram::SetNewLocalMeasurement( double distance,
                                                       double angle,
                                                       bool   dynamic,
                                                       int z_action)
    {

        unsigned id_phi = Angle2Sector( angle );

        if (dist_[id_phi] == double_inf)
        {
            // Is the first time that we receive a measurement on that direction
            dist_[id_phi] = distance;
        }
        else
        {
            // The sector was already filled before this entry
            // The system ponderates the measurements that are closer to the robot according to multi_input_pond_.
            double dist2mix_max = std::max(dist_[id_phi], distance);
            double dist2mix_min = std::min(dist_[id_phi], distance);

            // Originally multi_input_pond_ is bounded in the range [0,1]
            // We multiplied it by 0.5, so it will move on the range [0, 0.5]
            // pond = 0.0 (take only the closer measurement to the robot)
            //    pond' = 0.0
            //              (1.0 - 0.0)*dist2mix_min + (0.0)*dist2mix_max
            // pond = 1.0 (take them equally)
            //    pond' = 0.5
            //              (1.0 - 0.5)*dist2mix_min + (0.5)*dist2mix_max
            dist_[id_phi] = (1.0 - multi_input_pond_) * dist2mix_min + multi_input_pond_ * dist2mix_max;

            //std::cout << dist2mix_max << " " << dist2mix_min << " " << multi_input_pond_ << " " << dist_[id_phi] << std::endl;
        }


            // we receive information about height between UAVs

        switch(z_action){

            case Z_FREE:

            zdist_[id_phi] = 0;
            break;

            case Z_RANGE:
            zdist_[id_phi] = 1;
            break;

            case Z_SWAP:
            zdist_[id_phi] = 2;
            break;

        }


        // Dynamic measurements (true) take over static ones (false)
        dynamic_[id_phi] = dynamic_[id_phi] || dynamic;
    }

    /**
     * @brief Sets a measurement value in the POD, measured indirectly (E.g: information coming from the map)
     */
    void PolarObstacleDiagram::SetNewGlobalMeasurement( double x_robot,  double y_robot, double z_robot,  double yaw_robot,
                                                        double x_object, double y_object, double z_object, double r_object,
                                                        bool dynamic)
    {
        // Inflating the radious of the object before anything happens
        r_object += global_measurement_error_;

        // Centering the object in the position of the robot
        arma::colvec2 position;
        position(0,0) = x_object - x_robot;
        position(1,0) = y_object - y_robot;

        // Exchanging the angle to the robot's perspective
        arma::mat22 rotation;
        rotation(0,0) = +cos(-yaw_robot);
        rotation(0,1) = -sin(-yaw_robot);
        rotation(1,0) = +sin(-yaw_robot);
        rotation(1,1) = +cos(-yaw_robot);

        // Finding where the object is with respect to the robot
        arma::colvec pos_object = rotation * position;
        bool far_away = true;

        if (arma::norm(pos_object, 1) <= r_object)
        {
            // The circle invades the center of the robot. This should not happen normally
            far_away = false;
        }

        //Finding out the angle that points closer to the obstacle
        unsigned obstacle_id_phi = Angle2Sector( atan2(pos_object(1), pos_object(0)));

        //checking z distance
        int z_action=CheckZDistance(z_robot,z_object);


        if (far_away)
        {
            for (auto direction : {-1, 1})
            {
                unsigned guard  = 8;//id_phi_max_;   // Just in case that something wrong happens
                int id_phi      = obstacle_id_phi;
                double m,a,b,c;

                while (guard > 0)
                {
                    // The system should stop by far earlier than this number
                    --guard;

                    if (angle_[id_phi] != 0.0)  // Normal resolution, m is finite in y = m*x
                    {
                        m = tan(angle_[id_phi]);

                        /**
                         * Since the line is centered in the (0,0), b = 0
                         * We solve the following ecuation system:
                         *      y = m*x
                         *      (x - x_ob)^2 + (y - y_ob)^2 = r_ob^2
                         *      -------------------------------------
                         *      (1 + m^2)*x^2 + (-2*x_ob - 2*y_ob*m)*x + (-r_ob^2 + y_ob^2 + x_ob^2) = 0
                         *
                         * Solving it as a*x^2 + b*x + c = 0
                         */
                        a = (1.0 + pow( m, 2.0));
                        b = -2*pos_object(0) -2*pos_object(1)*m;
                        c = -pow(r_object, 2.0) + pow(pos_object(0), 2.0) + pow(pos_object(1), 2.0);
                    }
                    else // m is infinite in y = m*x
                    {
                        /**
                         * Since the line is centered in the (0,0), b = 0
                         * We solve the following ecuation system:
                         *      y = 0
                         *      (x - x_ob)^2 + (y - y_ob)^2 = r_ob^2
                         *      -------------------------------------
                         *      x^2 + (-2*x_ob)*x + (-r_ob^2 + y_ob^2 + x_ob^2) = 0
                         *
                         * Solving it as a*x^2 + b*x + c = 0
                         */
                        m = 0.0;
                        a = 1.0;
                        b = -2*pos_object(0);
                        c = -pow(r_object, 2.0) + pow(pos_object(0), 2.0) + pow(pos_object(1), 2.0);
                    }

                    double discriminant = pow(b, 2.0) - 4*a*c;

                    if (discriminant >= 0)
                    {
                        // There are two points hitting on the circle
                        double dist = double_inf;
                        for (double sign : {-1.0, +1.0})
                        {
                            double x_hit = (-b + sign*sqrt(discriminant))/(2*a);
                            double y_hit = m*x_hit;

                            dist = std::min( sqrt(pow(x_hit, 2.0) + pow(y_hit, 2.0)) , dist);
                        }
                        // if height is dangerous
                        if(z_action!=Z_FREE){

                            SetNewLocalMeasurement( dist, angle_[id_phi], dynamic, z_action);
                        }
                    }
                    else
                    {
                        // The circle will not be hit anymore
                        break;
                    }

                    id_phi = CircularIndex(id_phi + direction, id_phi_max_);
                }
            }
        }
        else    // the obstacle is too close
        {
            // This should never happen, so we are only creating a safety system
            // that introduces some information on the system
            double dist = arma::norm(pos_object, 1) - r_object;
            // information is introduced if height is dangerous
            if(z_action!=Z_FREE){
            SetNewLocalMeasurement( dist, angle_[obstacle_id_phi], dynamic, z_action);
            }
        }
    }

    /**
     * Analizes the full pod and finds the angles where a conflict exists.
     */
    bool PolarObstacleDiagram::GetConflicts(std::vector<double>& conflictive_angles, std::vector<int>& conflictive_heights)
    {
        BuildInflatedRegrion();
        conflictive_angles.clear();
        conflictive_heights.clear();

        std::vector<std::pair<double,int>> conflictive;
        // The system tryes to find  limits of a conflictive region and on it extracts
        // the local minimum.
        bool in_confl_region   = false;
        double   global_min_val = 0.0;
        unsigned global_min_idx = 0;

        for (unsigned id_phi = 0; id_phi < id_phi_max_; ++id_phi)
        {
            if ( !in_confl_region )
            {
                if ( infl_region_[id_phi] <= safety_region_[id_phi])
                {
                    in_confl_region = true;
                    global_min_val  = infl_region_[id_phi] - safety_region_[id_phi];
                    global_min_idx  = id_phi;
                }
            }
            else
            {   //in_confl_region
                if (  infl_region_[id_phi] > safety_region_[id_phi])
                {   // leaving the conflictive region
                    in_confl_region = false;
                    //conflictive_angles.push_back(angle_[global_min_idx]);
                    conflictive.push_back(std::make_pair(angle_[global_min_idx],zdist_[global_min_idx]));
                }
                else
                {   // finding out the local min
                    if ( infl_region_[id_phi] - safety_region_[id_phi] < global_min_val)
                    {
                        global_min_val  = infl_region_[id_phi] - safety_region_[id_phi];
                        global_min_idx  = id_phi;
                    }
                }
            }
        }

        if (in_confl_region)
        {
            // Conflict on the -PI/2 +PI/2 discontinuity
           conflictive.push_back(std::make_pair(angle_[id_phi_max_-1],zdist_[id_phi_max_-1]));
           // conflictive_angles.push_back(angle_[id_phi_max_-1]);

        }

        // Sorting the conflictive angles
        std::sort(conflictive.begin(), conflictive.end());

        for(unsigned id=0; id<conflictive.size();++id){
        conflictive_angles.push_back(conflictive[id].first);
        conflictive_heights.push_back(conflictive[id].second);
        }


        return conflictive_angles.size();
    }

    /* ****************************************************************************
     * SETTER                                                                     *
     * ****************************************************************************/
    /**
     * Defines the granularity of angles with measurements between -PI and PI
     */
    bool PolarObstacleDiagram::SetGranularity( const unsigned granularity)
    {
        if ( granularity >= 16 )
        {
            id_phi_max_ = granularity;
            ReservePODVectors();
            return true;
        }
        else
        {
            return false;
        }
    }

    /**
     * Defines the number of measuremets per direction
     */
    bool PolarObstacleDiagram::SetMeasurementsPerDirection( const unsigned num_measurements)
    {
        if ( num_measurements > 0 )
        {
            id_r_max_ = num_measurements;
            ReservePODVectors();
            return true;
        }
        else
        {
            return false;
        }
    }

    /**
     * Defines the way that the measurements per direction are going to be ponderated
     */
    bool PolarObstacleDiagram::SetMultiInputPond( const double multi_input_pond)
    {
        if (0.0 <= multi_input_pond && multi_input_pond <= 1.0)
        {
            multi_input_pond_ = 0.5*multi_input_pond;
            return true;
        }
        else
        {
            return false;
        }
    }

    /**
     * Set a new value for the safety_region
     */
    void PolarObstacleDiagram::SetSafetyRegion( const double r_safety_region )
    {
        if (r_safety_region > 0.0 && id_phi_max_)
        {
            for (unsigned id_phi = 0; id_phi < id_phi_max_; ++id_phi)
            {
                double angle = +M_PI -id_phi*2*M_PI/double(id_phi_max_);
                r_safety_set_ = SetSafetyRegion( r_safety_region, angle);
            }
        }
    }

    /**
     * Sets the BrackingDistance inside the POD
     */
    void PolarObstacleDiagram::SetBrackingDistance( const double bracking_distance)
    {
        if (bracking_distance > 0.0)
        {
            bracking_distance_ = bracking_distance;
        }
    }
    /**
     * Sets the Dzmin inside the POD
     */
    void PolarObstacleDiagram::SetDzmin( const double dz_min)
    {
        if (dz_min > 0.0)
        {
            dz_min_ = dz_min;
        }
    }
    /**
     * Sets the Dzrange inside the POD
     */
    void PolarObstacleDiagram::SetDzrange( const double dz_range)
    {
        if (dz_range > 0.0)
        {
            dz_range_ = dz_range;
        }
    }
    /**
     * Sets the Local Measurement Error inside the POD
     */
    void PolarObstacleDiagram::SetLocalMeasurementError( const double error)
    {
        if (error >= 0.0)
        {
            local_measurement_error_ = error;
        }
    }

    /**
     * Sets the Global Measurement Error inside the POD
     */
    bool PolarObstacleDiagram::SetGlobalMeasurementError( const double error)
    {
        /*
         * This function uses 2*error instead of error. A global localization system not only
         * introduces error in the sender but also on the receiver.
         */
        if (error >= 0.0 && local_measurement_error_ >= 0.0)
        {
            if (local_measurement_error_ > 2*error)
            {
                /*
                 * This thing should never happen. Global communication systems are
                 * allways less reliables than local ones. In case that this happens,
                 * we approximate the global_measurement_error with the local one.
                 */
                global_measurement_error_ = 0.0;
            }
            else
            {
                /*
                 * The local measurement error is going to be taken into account always,
                 * therefore, we are substracting it here
                 */
                global_measurement_error_ = 2*error - local_measurement_error_;
            }
            return true;
        }
        return false;
    }



    /**
     * Sets the GammaOffset inside the POD
     */
    void PolarObstacleDiagram::SetGammaOffset( const double offset)
    {
        if (offset >= 0.0)
        {
            gamma_offset_ = offset;
        }
    }

    /**
     * Sets the smooth factor
     */
    void PolarObstacleDiagram::SetSmoothFactor( const unsigned num_measurements)
    {
        if (num_measurements > 1)   //otherwise makes no sense
        {
            smooth_factor_ = num_measurements;
            X_.set_size(num_measurements, POL_DEG + 1);
            Y_.set_size(num_measurements, 1);

            for (unsigned row = 0; row < num_measurements; ++row)
            {
                X_(row, 0) = 1.0;
            }
        }
        else if (num_measurements >= 0)
        {
            smooth_factor_ = 0;
        }
    }

    /**
     * Returns all the information belonging to a specific direction of the space
     */
    PolarObstacleDiagram::measurements PolarObstacleDiagram::GetMeasurement( const unsigned id_phi)
    {
        measurements tmp = {0.0, 0.0, false, 0.0};

        if (id_phi < id_phi_max_)
        {
            tmp.dist    = std::max( infl_region_[id_phi], 0.0);
            tmp.angle   = angle_[id_phi];
            tmp.dynamic = dynamic_[id_phi];
            tmp.safety_region = safety_region_[id_phi];
        }

        return tmp;
    }

    /**
     * Debugging function that prints the POD
     */
    void PolarObstacleDiagram::Print()
    {
        std::cout << std::fixed;
        std::cout << std::setprecision(2);

        for (int id_r = id_r_max_ - 1; id_r >= 0; --id_r)
        {
            for (unsigned id_phi = 0; id_phi < id_phi_max_; ++id_phi)
            {
                if (dist_[id_phi + id_r*id_phi_max_] > 9.99)
                {
                    std::cout << " | " << ">9.9";
                }
                else
                {
                    std::cout << " | " << dist_[id_phi + id_r*id_phi_max_];
                }
            }
            std::cout << std::endl;
        }

        // Printing a line between the angles and the measurements
        for (unsigned id_phi = 0; id_phi < id_phi_max_; ++id_phi)
        {
            std::cout << "-+-----";
        }
        std::cout << std::endl;

        // Printing the inflated region
        for (unsigned id_phi = 0; id_phi < id_phi_max_; ++id_phi)
        {
            if (infl_region_[id_phi] > 9.99)
            {
                std::cout << " | " << ">9.9";
            }
            else
            {
                std::cout << " | " << infl_region_[id_phi];
            }
        }
        std::cout << std::endl;

        // Printing a line between the angles and the measurements
        for (unsigned id_phi = 0; id_phi < id_phi_max_; ++id_phi)
        {
            std::cout << "-+-----";
        }
        std::cout << std::endl;


        // Printing the angles as a reference
        for (unsigned id_phi = 0; id_phi < id_phi_max_; ++id_phi)
        {
            if (angle_[id_phi] < 0.0)
            {
                std::cout << " |" << angle_[id_phi];
            }
            else
            {
                std::cout << " | " << angle_[id_phi];
            }
        }
        std::cout << std::endl;
        std::cout << std::endl;
    }

    /* ****************************************************************************
     * Inheritable utility methods                                                *
     * ****************************************************************************/

    /**
     * Utility function. Returns an angle between (-pi and pi)
     */
    double PolarObstacleDiagram::ScaleAngle(double angle)
    {
        while (angle < -M_PI)
        {
            angle += 2.0*M_PI;
        }
        while (angle > +M_PI)
        {
            angle -= 2.0*M_PI;
        }
        return angle;
    }

    /**
     * Converts an angle to its specific sector index.
     */
    unsigned PolarObstacleDiagram::Angle2Sector( double angle)
    {
        // Computes the best aproximation to the sector
        angle = ScaleAngle( angle );
        int id_phi = roundl( (-angle/M_PI + 1)*(id_phi_max_/2 ) );

        // Forcing the approximation to be inside the vector limits
        id_phi = std::max(0, id_phi);
        id_phi = std::min(id_phi, int(id_phi_max_-1));

        return unsigned(id_phi);
    }

    /**
     * Returns an index value on the vector as if the vector where circular.
     */
    int PolarObstacleDiagram::CircularIndex( int idx, int idx_max)
    {
        while (idx < 0)
        {
            idx += idx_max;
        }
        while (idx >= idx_max)
        {
            idx -= idx_max;
        }

        return idx;
    }

    /**
     * The function returns the difference between the desited avoidance distance and the real distance
     */
    double PolarObstacleDiagram::GetYawAvoidanceDistanceError( double angle_conflict)
    {
        unsigned id_phi = Angle2Sector(angle_conflict);
        /*  Tryes to remain at bracking_distance_/2 if the object is static
         * or at bracking_distance if the object is dynamic
         *
         *      |Robot|             |Objet|
         *             <----------->
         *          This is the distance to keep
         */
        double pond = 2.0;  // For static obstacles we only take one time the bracking distance
        if (GetDynMeasurement(id_phi))
        {
            pond = 1.0;     // For dynamic obstacles we take two times the bracking distance
        }
        double desired_dist =  pond * (bracking_distance_ + gamma_offset_ + local_measurement_error_);  // 0.5 *

        double current_dist = GetDistMeasurement( id_phi) + safety_region_[id_phi];

        // Returns positive values to request an atraction and
        // negative values to request a repulsion
        double error = -(current_dist - desired_dist)/desired_dist;



        /* bool do_not_glue = true;

        if (do_not_glue)
        {
            error = std::max(error, 0.0);
        } */

        return error;
    }

    /* ****************************************************************************
     * Internal methods                                                           *
     * ****************************************************************************/

    /**
     * Reserves space for all necessary measurements
     */
    bool PolarObstacleDiagram::ReservePODVectors()
    {
        if (id_phi_max_ * id_r_max_ > 0)
        {
            // Cleaning possible old values
            dist_.clear();
            angle_.clear();
            zdist_.clear();
            dynamic_.clear();
            safety_region_.clear();
            infl_region_.clear();
            tmp_.clear();

            // Allocating all memory in a row and initalizing it
            dist_.assign(id_phi_max_ * id_r_max_, double_inf);
            zdist_.assign(id_phi_max_ * id_r_max_, 0);
            angle_.reserve(id_phi_max_);
            dynamic_.assign(id_phi_max_ * id_r_max_, false);
            safety_region_.assign(id_phi_max_, double_inf);
            infl_region_.assign(id_phi_max_, double_inf);
            tmp_.assign(id_phi_max_, double_inf);

            for (unsigned id_phi = 0; id_phi < id_phi_max_; ++id_phi)
            {
                // All is static unless someone specifies the oposite.
                angle_.push_back(+ M_PI - (id_phi * 2 * M_PI / id_phi_max_ ));
            }
        }
    }

    /**
     * Build an inflated version of the measurements around the robot.
     */
    void PolarObstacleDiagram::BuildInflatedRegrion()
    {
        // Building the inflated region without filtering
        for (unsigned id_phi = 0; id_phi < id_phi_max_; ++id_phi)
        {
            infl_region_[id_phi] = GetDistMeasurement(id_phi);
        }

        if (smooth_factor_ > 1)
        {
            // Filtering the measurements
            for (int id_phi = 0; id_phi < id_phi_max_; ++id_phi)
            {
                bool inf_found = false;

                // Applying polinomial regression to make the measurements fit under a curve y=ax^2 + bx + c
                for (unsigned row = 0; row < smooth_factor_; ++row)
                {
                    int id_aux = id_phi - round(double(smooth_factor_)/2.0) + row;
                    Y_(row, 0) = infl_region_[CircularIndex(id_aux, id_phi_max_)];

                    if (Y_(row, 0) == double_inf)
                    {
                        inf_found = true;
                        break;
                    }

                    for (unsigned col = 1; col <= POL_DEG; ++col)
                    {
                        X_(row, col) = powf( angle_[CircularIndex(id_aux, id_phi_max_)], double(col));
                    }
                }

                if (inf_found)
                {
                    // Filter does not apply with infinite values
                    tmp_[id_phi] = infl_region_[id_phi];
                }
                else
                {
                    arma::mat a = pinv(trans(X_)*X_)*trans(X_)*Y_;
                    arma::rowvec x(1, POL_DEG + 1);
                    x(0,0) = 1.0;
                    for (unsigned col = 1; col <= POL_DEG; ++col)
                    {
                        x(0, col) = powf( angle_[id_phi], double(col));
                    }

                    arma::mat y = x*a;
                    tmp_[id_phi] = y(0,0);
                }
            }

            infl_region_.swap(tmp_);
        }
    }

    /**
     * Returns the combined measurement placed on the angle of index id_phi
     */
    double PolarObstacleDiagram::GetDistMeasurement( unsigned id_phi)
    {
        double measurement = double_inf;
        bool   dynamism    = false;
        if (id_phi < id_phi_max_)
        {
            for (unsigned id_r = 0; id_r < id_r_max_; ++id_r)
            {
                measurement = std::min(measurement, GetDistMeasurement( id_phi, id_r));
                dynamism    = dynamism || GetDynMeasurement( id_phi, id_r);
            }
        }

        double pond = 1.0;
        if (dynamism)
        {
            pond = 2.0;
        }
        measurement -= pond*bracking_distance_;
        measurement -= pond*local_measurement_error_;      // Includes even more safety
        measurement -= pond*gamma_offset_;

        return measurement;
    }

    /**
      * Returns info about Z distance
      */

    int PolarObstacleDiagram::CheckZDistance(double z_robot, double z_object)
    {
        double difference=fabs(z_robot-z_object);
        double range_difference=dz_min_+dz_range_;

        if(dz_min_ > difference){
          return Z_SWAP;
        }
        if(range_difference > difference){

          return Z_RANGE;
        }
        if(difference> range_difference)
        {
          return Z_FREE;

        }
    }

    /**
     * Returns the combined measurement of dynamism on the angle of index id_phi
     */
    bool PolarObstacleDiagram::GetDynMeasurement( unsigned id_phi)
    {
        bool   dynamism    = false;
        if (id_phi < id_phi_max_)
        {
            for (unsigned id_r = 0; id_r < id_r_max_; ++id_r)
            {
                dynamism  = dynamism || GetDynMeasurement( id_phi, id_r);
            }
        }
        else
        {
            // If there is an error, better dynamic
            std::cout << "Error while testing dynamism" << std::endl;
            dynamism = true;
        }

        return dynamism;
    }

    /**
     * getMeasurement returns the value of the measurement id_phi,id_r
     */
    double PolarObstacleDiagram::GetDistMeasurement( unsigned id_phi, unsigned id_r)
    {
        id_phi = std::min(id_phi, id_phi_max_ - 1);
        id_r   = std::min(id_r,   id_r_max_   - 1);

        return dist_[id_phi + id_r*id_phi_max_];
    }

    /**
     * getDynMeasurement returns the value of dynamism in id_phi,id_r
     */
    bool PolarObstacleDiagram::GetDynMeasurement( unsigned id_phi, unsigned id_r)
    {
        id_phi = std::min(id_phi, id_phi_max_ - 1);
        id_r   = std::min(id_r,   id_r_max_   - 1);

        return dynamic_[id_phi + id_r*id_phi_max_];
    }

    /**
     * Setter that helps to define the Safety Region around the robot
     */
    bool PolarObstacleDiagram::SetSafetyRegion( const double dist, const double angle)
    {
        if (dist > 0.0 && safety_region_.size())
        {
            unsigned id_phi = Angle2Sector(angle);
            safety_region_[ id_phi ] = dist;
        }
        // Returning if all possible direction was fully set
        if (safety_region_.size())
        {
            return (*std::max_element(safety_region_.begin(), safety_region_.end()) != double_inf);
        }
        return false;
    }

}   // namespace avoid
