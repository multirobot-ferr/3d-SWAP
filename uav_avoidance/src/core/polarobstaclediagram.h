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
 * @file    polarobstaclediagram.h
 * @author  Eduardo Ferrera
 * @date    January, 2017
 * @brief   Contains the Polar Obstacle Diagram manager
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

#ifndef SWAP_POLAROBSTACLEDIAGRAM_H_
#define SWAP_POLAROBSTACLEDIAGRAM_H_

// Including all other requirements
#include <vector>
#include <math.h>       // Matematical operations
#include <limits>       // std::numeric_limits
#include <algorithm>    // std::rotate
#include <iostream>     // std::string
#include <iomanip>      // std::setprecision  (to debug)
#include <armadillo>    // matematical library, requires:
                        //  "sudo apt-get install libarmadillo-dev libblas-dev liblapack-dev"

namespace avoid
{
    #define POL_DEG 1       // Degree of the polinomium to perform approximations in the measurements
                            // I tryed with higher polinomium degrees but the performance is better with this one

    /**
     * @brief Manager of the Polar Obstacle diagram
     *
     * Maps out the distances to obstacles relative to the current position and orientation of the robot.
     *
     * The whole SWAP works over the information that is provided there.
     *           distance
     *        X X  / \        X
     *    X   . .   |   X   X .
     *    . X . . X | X .   . . X
     *  X . . . . . | . . X . . . X
     *  . . . . . . | . . . . . . .
     * -----------------------------> theta
     * +PI          0             -PI
     * Note:  theta = 0       -> Front of your robot
     *        theta = +PI/2   -> Left part of your robot
     *        theta = -PI/2   -> Right part of your robot
     */
    class PolarObstacleDiagram
    {
        public:
            /**
             * @brief Default constructor
             */
            PolarObstacleDiagram();

            /**
             * @brief Default destructor
             */
            virtual ~PolarObstacleDiagram();

            /**
             * @brief Structure that helps to print the full POD
             */
            struct measurements
            {
                double dist;            //<[m] Distances measured on a specific direction
                double angle;           //<[rad] (with respect to the front of the robot)
                bool   dynamic;         //<If the object is known to be static or dynamic (true if not known)
                double safety_region;   //<[m] Region where the robot is defined
            };

            /* ****************************************************************************
             *   MAIN METHODS (Intended to control the behaviour of the POD)              *
             * ****************************************************************************/

            /**
             * @brief IsReady returns if the POD is well configurated or not. And if not, explains why.
             * @param[out] (std::string)why_not returns by reference why is swap not ready
             * @return Returns true if swap is ready
             */
            bool IsReady(std::string& why_not);

            /**
             * @brief Forgets the oldest measurement set in order to receive a new one
             *
             * This function should be called every time that a new dataset comes. Otherwise, the
             * measurements will be merged with the current existing data.
             * Example: call it each time that your laser gives you a new measurement set
             */
            void NewMeasurementSetReceived();

            /**
             * @brief Sets a measurement value in the POD, measured directly on the robot (E.g: with a ranger)
             *
             * This function rounds the specified angle to an angle inside of the granularity of the POD
             * and merges the current value of that measurement with the incoming one.
             * @param distance The distance between the center of the robot and the obstacle
             * @param angle    Angle [rad] with respect to the front of the robot (positive values are left side of the robot)
             * @param dynamic  If the measurement belongs to a dynamic obstacle or not.
             */
            void SetNewLocalMeasurement(double distance,
                                        double angle,
                                        bool   dynamic = false);

            /**
             * @brief Sets a measurement value in the POD, measured indirectly (E.g: information coming from the map)
             *
             * This function fills all measurements of the POD corresponding to a round object located in x_object,y_object
             * with a radius r_object.
             * @param x_robot Position x[m] of the robot with respect to the map
             * @param y_robot Position y[m] of the robot with respect to the map
             * @param yaw_robot Orientation yaw[rad] of the robot with respect to the map
             * @param x_object Position x[m] of the object with respect to the map
             * @param y_object Position y[m] of the object with respect to the map
             * @param r_object Radius[m] of the object (system assumes a circular object)
             * @param dynamic Boolean to determine if the obstacle is static or dynamic
             */
            void SetNewGlobalMeasurement(double x_robot,  double y_robot,  double yaw_robot,
                                         double x_object, double y_object, double r_object,
                                         bool dynamic = true);

            /**
             * @brief Analizes the full pod and finds the angles where a conflict exists.
             * @param[out] conflictive_angles returns by reference all the conflictive angles.
             * @return Returns true if there is at least one conflictive direction
             */
            bool GetConflicts(std::vector<double>& conflictive_angles);


            /* ****************************************************************************
             * SETTER                                                                     *
             * ****************************************************************************/
            /**
             * @brief Defines the number of angles with measurements between -PI and PI
             *
             * Reserves memory for measurements incresing in steps of (2*PI/granularity) rads.
             * @param granularity number of angles measurements with measurements between -PI and PI (min 16)
             * @note Cleans the information contained on the vector
             * @return Success of the operation
             */
            bool SetGranularity( const unsigned granularity);

            /**
             * @brief Defines the number of measuremets per direction
             *
             * On each (2*PI/granularity) rads step defines num_measurements measurements to track.
             * @note Cleans the information contained on the vector (min 1)
             * @return Success of the operation
             */
            bool SetMeasurementsPerDirection( const unsigned num_measurements);

            /**
             * @brief Defines the way that the measurements per direction are going to be ponderated
             * @param multi_input_pond parameter between 0.0 and 1.0 that ponderates more or less the values closer to the robot in a direction
             * @return  Success of the operation
             */
            bool SetMultiInputPond( const double multi_input_pond);

            /**
             * @brief Setter that helps to define the Safety Region around the robot
             *
             * Sets a circle of radious r_sr centered in the middle of the robot and enconpassing its whole contour.
             * @param r_safety_region Set new value to safety_region (r_safety_region > 0.0)
             * @return whether or not are all the measurements filled.
             */
            void SetSafetyRegion( const double r_safety_region );

            /**
             * @brief Sets the BrackingDistance inside the POD
             *
             * The bracking distance is the distance required to fully stop a robot when
             * moving at full speed.
             * @param Value of the bracking_distance
             */
            void SetBrackingDistance( const double bracking_distance);

            /**
             * @brief Sets the Local Measurement Error inside the POD
             *
             * The Local Measurement Error is the maximum error that a single measurement a local device (like your ranger)
             * could have.
             * @param Value of the local_measurement_error
             * @note This value inflates even more the obstacles
             */
            void SetLocalMeasurementError( const double error);

            /**
             * @brief Sets the Global Measurement Error inside the POD
             *
             * The Global Measurement Error is the maximum error that a single measurement in a global measurement
             * device (like the positioning error when you share your position) can have.
             * @param Value of the global_measurement_error
             * @return Success of the operation
             * @note This value inflates even more the obstacles
             * @note The local_measurement_error should be set first.
             */
            bool SetGlobalMeasurementError( const double error);

            /**
             * @brief Sets the GammaOffset inside the POD
             *
             * The GammaOffset is an offset introduced on the system to take into account potential
             * errors, communication delays etc.
             * @param Value of the offset
             */
            void SetGammaOffset( const double offset);

            /**
             * @brief Sets the smooth factor
             *
             * Control the number of measurements that the system uses to interpolate a laser measurement with noise.
             * Helps to fight agains noise.
             * @param Number of measurements (recomemded 5)
             */
            void SetSmoothFactor( const unsigned num_measurements);

            /* ****************************************************************************
             * GETTER                                                                     *
             * ****************************************************************************/


            /**
             * @brief Returns all the information belonging to a specific direction of the space
             * @param id_phi id of the specific direction of the space
             * @return Struct with all this information
             */
            measurements GetMeasurement( const unsigned id_phi);

            /* ****************************************************************************
             * Other methods                                                              *
             * ****************************************************************************/
            /**
             * @brief Debugging function that prints the POD
             */
            void Print();

        protected:
            // Members used in a class that inherits.
            unsigned id_phi_max_ = 0;           //<[#] Granularity of the measurements between -PI and PI

            /* ****************************************************************************
             * Inheritable utility methods                                                *
             * ****************************************************************************/

            /**
             * @brief Utility function. Returns an angle between (-pi and pi)
             *
             * @param angle Angle in radians
             * @return Same angle but inside of the range (-pi,pi).
             */
            double ScaleAngle(double angle);

            /**
             * @brief Utility function. Converts an angle to its specific sector index.
             * @param angle Angle to convert to a sector index
             */
            unsigned Angle2Sector( double angle);

            /**
             * @brief Returns an index value on the vector as if the vector where circular.
             * @param idx index of the vector. Can hit out of range values.
             * @param idx_max maximum value of the vector.
             */
            int CircularIndex( int idx, int idx_max);

            /**
             * @brief The function returns the difference between the desited avoidance distance and the real distance in a normalized way
             *
             * Swap tryes to keep a distance between obstacles. In order to keep this distance, the error of distance
             * should be measured. This error is returned normalized.
             * @param angle_conflict Angle where a conflict was previously located
             * @return Error in distance.
             */
            double GetYawAvoidanceDistanceError( double angle_conflict);

            std::vector<double> infl_region_;   //<[m] Distances inflated and filtered.
            std::vector<double> angle_;         //<[rad] (with respect to the front of the robot)

        private:
            std::vector<double> dist_;          //<[m] Distances measured on a specific direction
            std::vector<bool>   dynamic_;       //<If the object is known to be static or dynamic (true if not known)
            std::vector<double> safety_region_; //<[m] Region where the robot is defined
            std::vector<double> tmp_;           //<[m] Temporal movement of measurements

            unsigned id_r_max_     = 0;         //<[#] Number of measurements per angle
            double   multi_input_pond_  = -1.0; //<[#] Ponderation between measurements [0.0,1.0]
            bool     r_safety_set_ = false;     //!< Tracks if r_safety_region was set or not
            double   bracking_distance_ = +0.0; //<[m] Distance to fully stop a robot (larger than zero)
            double   local_measurement_error_ = -1.0;  //<[m] Maximum possible error of a local measurement (like a ranger measurement)
            double   global_measurement_error_ = -1.0; //<[m] Maximum possible error of a global measurement (like the one comming from a communication system)
            double   gamma_offset_ = -1.0;      //<[m] Offset to take into account other potential errors

            // Smoothing variables
            int      smooth_factor_ = -1;       //!< Number of measurements to interpolate to filter out some noise
            arma::mat X_;
            arma::mat Y_;

            const double double_inf = std::numeric_limits<double>::max(); //<Infinite value for a measurement

            /* ****************************************************************************
             * Internal methods                                                           *
             * ****************************************************************************/

            /**
             * @brief Reserves space for all necessary measurements
             *
             * It uses the values id_phi_max_ and id_r_max_ to reserve space in all vectors
             * and fills them with clean values
             * @return Success of the operation
             */
            bool ReservePODVectors();

            /**
             * @brief Build an inflated version of the measurements around the robot.
             *
             * Creates a vector of measurements around the robot in a safe way.
             */
            void BuildInflatedRegrion();

            /**
             * @brief Returns the combined measurement placed on the angle of index id_phi
             *
             * It computes the inflated measurement on a specific direction taking into account all
             * historical values on that direction.
             * @param id_phi Index of an angle in the array
             * @return Combined measurement (filtered out) of the measurements on that direction
             */
            double GetDistMeasurement( unsigned id_phi);

            /**
             * @brief Returns the combined measurement of dynamism on the angle of index id_phi
             * @param id_phi Index to evaluate
             * @return Dynamism measured on that direction
             */
            bool GetDynMeasurement( unsigned id_phi);

            /**
             * @brief getDistMeasurement returns the value of the measurement id_phi,id_r
             * @param id_phi Index for the phi element of the vector
             * @param id_r Index for the r elemen of the vector
             * @return value of the measurement id_phi,id_r
             */
            double GetDistMeasurement( unsigned id_phi, unsigned id_r);

            /**
             * @brief getDynMeasurement returns the value of dynamism in id_phi,id_r
             * @param id_phi Index for the phi element of the vector
             * @param id_r Index for the r elemen of the vector
             * @return value of the dynamism in id_phi,id_r
             */
            bool GetDynMeasurement( unsigned id_phi, unsigned id_r);

            /**
             * @brief Setter that helps to define the Safety Region around the robot
             *
             * The safety region should overcome the full shape of the robot.
             * @param dist Distance of the Safety Region at a specific angle
             * @param angle Specifies an angle around the robot
             * @return whether or not are all the measurements filled.
             */
            bool SetSafetyRegion( const double dist, const double angle);


    };

}; //namespace avoid

#endif // SWAP_POLAROBSTACLEDIAGRAM_H_
