/*
 * Copyright (c) 2017, swap-ferr
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
 * @file    swap.h
 * @author  Eduardo Ferrera
 * @date    January, 2017
 * @brief   Contains the implementation of the Swap collision avoidance system
 *
 * This code implements SWAP: safety-enhanced avoidance policy. Such policy
 * establish a behaviour for a team of UGV in order to avoid conflict between them
 * while they move to their destiny.
 * Convergence of the full system is only ensured under certain conditions.
 *
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

#ifndef SWAP_SWAP_H_
#define SWAP_SWAP_H_

// Macros
#define RAD2DEG(X) 180*X/M_PI

// Headers
/*
 * Including other required headers.
 * The polarobstaclediagram.h contains a class capable of tracking all
 * measurements that a laser-ranger reader with noise sends to swap and
 * able to filter out some of the noise using a filtering system.
 */
#include "polarobstaclediagram.h"

namespace avoid
{

    class Swap: public PolarObstacleDiagram
    {
        public:
            /**
             * @brief Constructor of the class
             *
             * Initializes all the variables
             */
            Swap();

            /** Default destructor */
            virtual ~Swap();

            /**
             * @brief IsReady returns if swap is well configurated or not. And if not, explains why.
             * @param[out] (std::string)why_not returns by reference why is swap not ready
             * @return Returns true if swap is ready
             */
            bool IsReady(std::string& why_not);

            /* ****************************************************************************
             * MAIN FUNCTIONS (Intended to be executed on the main loop of the robot)     *
             * ****************************************************************************/
            /**
             * @brief Set the position of the goal with respect to the robot (in polars)
             *
             * Swap navigate making use of the distance to the goal and the angle.
             * The algorithm will follow this reference unless there are obstacles
             * creating a conflicts
             * @param distance The distance to the goal
             * @param angle    Angle with respect to the front of the robot to the goal (in radians).
             */
             void SetGoal( double distance, double angle );

             /**
              * @brief Calculates a direction and a speed for collision avoidance.
              *
              * Returns via parameters the requested orientation and speed according to the
              * current avoidance situation. v_ref is normalized between 0.0 and 1.0. Please
              * multiply it the maximum speed of your robot.
              * \param[out] v_ref The reference velocity for collision avoidance (normalized between 0 and 1)
              * \param[out] yaw_ref The reference orientation for collision avoidance (relative to the current orientation, positive value means go left).
              */
             void CollisionAvoidance( double& v_ref, double& yaw_ref );

            /* ****************************************************************************
             * SETTER                                                                     *
             * ****************************************************************************/

             /**
              * @brief Configures the Goal Lateral Vision parameter
              *
              * The goal lateral vision is a parameter between 360 & 10 deg that makes the system ignore where the
              * conflicts are placed unless they are infront of the robot in a field of view of that size.
              * This allows the system to rotate around s convex conflict (if is set at less than 90 deg) surrounding
              * those objects in the same way that a bug algorithm will do.
              * Recommended value, 90 deg
              * @param goal_lateral_vision value
              */
             void SetGoalLateralVision( double goal_lateral_vision);

             /**
              * @brief Informs to swap if the robot is holonomic (false configured as default)
              *
              * Swap tryes to reduce the speed if the robot is not well oriented to the desired
              * direction. This does not happens if holonomic is set to true
              * @param holonomic_robot value
              */
             void SetHolonomicRobot( bool holonomic_robot);

             /**
              * @brief Configures the value of the rotation control proportional
              *
              * This value is used to make the robots remain at certain distance of
              * the obstacles arround while avoiding. This means that can drive away
              * if they are too close, but also drive to the obstacle if is too far.
              * Acts like the P of a PID
              * @brief rotation control proportional value
              */
             void SetRotCtrlP( double rot_ctrl_P);

             /* ****************************************************************************
              * GETTER                                                                     *
              * ****************************************************************************/
             /**
              * @brief Returns the state of the state machine
              *
              * Returns a number that represents the state of the state machine.
              * Moreover, if a string is given, returns on that string the name
              * of the state in the state machine.
              * @param[out] state_name returns the name of the state in the state machine
              * @return 1 for Free, 2 for Rendezvous, 3 for Rencontre and 4 for Blocked
              */
             unsigned GetMachineState( std::string& state_name );

             /**
              * @brief Returns the state of the state machine if there is a change since last call
              *
              * Checks the current and last state of the state machine and if they are different
              * it returns something else than 0. See GetMachineState.
              * @param[out] state_name returns the name of the state in the state machine. Empty if no changes
              * @return 0 for no changes, 1 for Free, 2 for Rendezvous, 3 for Rencontre and 4 for Blocked
              */
             unsigned GetMachineStateChanges( std::string& state_name );

        protected:
            std::vector<double> conflictive_angles_; //!< Storage to place the conflicts found.

            /** State machine for the orientation of the robot.
             * FREE:        There are no obstacles, or they can be ignored.
             * RENDEZVOUS:  A non-ignorable obstacle is met. The robot should stop while prepare themselves to surround it.
             * RENCONTRE:   The robot surrounds the not-ignorable obstacle.
             * BLOCKED:     Surrounding obstacles (other robots mainly) trapped the robot. Wait here.
             */
            enum state_orientation {FREE = 1,
                                    RENDEZVOUS = 2,
                                    RENCONTRE = 3,
                                    BLOCKED = 4};

            /**
             * Options for the default behaviour.
             * COUNTERCLOCKWISE: Always try to surround an obstacle by going counter-clockwise around it.
             * TOGOAL: Try to rotate towards the goal.
             * CLOCKWISE: Surround clockwise.
             */
            enum rot_behaviour {COUNTERCLOCKWISE = +1,
                                CLOCKWISE = -1,
                                NOTDEFINED = 0};

        private:
            // Variables to track if the system is ready
            bool is_ready_ = false;                 //!< Tracks if all values of Swap are initialized.

            // Goal storage
            double goal_dist_ = 0.0;                //!< Distance to the goal
            double goal_angle_= 0.0;                //!< Angle to the goal (in radians)

            /* ****************************************************************************
             * Internal parameters for SWAP                                               *
             * ****************************************************************************/
            state_orientation statusOri_ = FREE;    //!< Current state for the orientation state machine.
            double yaw_avoidance_ = 0.0;            //!< Orientation to follow on the RENDEZVOUS or RENCONTRE cases
            double v_avoidance_   = 0.0;            //!< Speed to follow on the RENDEZVOUS case

            // Conflict dealing
            double goal_lateral_vision_ = 0.0;      //!< Instead of looking for the goal in the entire navigable area, in order to define an obstacle as ignorable or not, looks only in an area defined by this parameter in radians. Allows to deal with convex-walls conflicts.
            double rot_ctrl_P_ = 0.0;               //!< Acts as a P controller trying to keep the distance while surround other obstacles.
            double yaw_max_err_ = 10*M_PI/180.0;    //!< Maximal error allowed in RENCONTRE state
            double lin_v_rendezvous_ = 1.0;         //!< Maximal speed in the rendezvous state
            rot_behaviour rot_behaviour_ = COUNTERCLOCKWISE;           //!< +1: Always rotate counter-clockwise, 0: not defined; -1: Always rotate clockwise

            // Goal dealing
            double d_goal_ = 0.1;                   //!< Distance where swap assumes that you are on the goal
            double d_approach_ = 0.3;               //!< Distance close to the goal where the robot should start braking.

            // Other variables
            /**
             * The robot will try to stop if his orientation is too far away from the desired one
             * unless the robot is holonomic
             */
            bool holonomic_robot_ = false;

            /**
             * @brief Utility function Manages the conflicts and if a solution exists fills the references
             *
             * Checks the direction where all conflicts comes. If there are ways to avoid those conflicts,
             * it fills the variables yaw_avoidance_ and v_avoidance_ to indicate that.
             * It also changes the value of statusOri_ indicating the current status of the state machine.
             */
            void ConflictsManager();

            /**
             * @brief Computes all the forbidden direction due to the inflated safety regions
             * @param conflict_left_phi Orientation to the conflict that allows avoidance to the left
             * @param conflict_right_phi Orientation to the conflict that allows avoidance to the right
             * @return returns true if there exists an avoidance solution to the conflict.
             */
            bool ComputeForbiddenDirections( double& conflict_left_phi, double& conflict_right_phi);

            /**
             * @brief Utility function. Checks if all found conflicts can be ignored.
             *
             * Checks explicitly for the following cases:
             * - if the robot is facing the navigable area
             * - if the goal is within the navigable area
             * - if the goal is visible according to the angle of view to ignore
             * If all conditions are met, the function returns true.
             *
             * @param Angle where the left  conflict was found
             * @param Angle where the right conflict was found
             * \param goalAngle Angle to the goal relative to the robot's current orientation. Positive if the goal is to the left.
             * \param goal_lateral_vision How far to the later is the robot able to see (used to avoid some deadlocks like for U-Shapes)
             * \return True if the conflict is ignorable.
             */
            bool AreConflictsIgnorable( const double conflict_left_phi,
                                        const double conflict_right_phi,
                                        const double goal_angle,
                                        const double goal_lateral_vision_);

            /**
             * @brief Computes for the RENDEZVOUS or RENCONTRE states the necessary avoidance directions
             *
             * When a robot finds one or multiple non-ignorable conflicts, this function
             * determines the directions that the robot has to follow in order to avoid such
             * conflicts in the most efficient way.
             * @param conflict_left_phi the left conflict
             * @param conflict_right_phi the right conflict
             * @note it also aplyes a P controler to keep the rotation distance.
             */
            void ApplyAvoidancePolicy( const double conflict_left_phi,
                                       const double conflict_right_phi);

            /**
             * @brief Returns the speed that the robot should have according to the situation
             *
             * The system increases or decreases the speed depending on how far the robot is from
             * his goal, and how far is from its desired orientation.
             * @note Only apply it on the FREE state
             * @return Desired velocity
             */
            double GetDesiredVelocity();

            /**
             * @brief Utility function. Computes the difference between two orientation angles
             *
             * Utility function. Computes the difference between two orientation angles
             * taking into account the -pi/pi discontinuity.
             *
             * @param ang1 The first angle (between -pi and pi).
             * @param ang2 The second angle (between -pi and pi).
             * @return The difference between ang1 and ang2.
             */
            double AngleDiff(double ang1, double ang2);

            /**
             * @brief Utility function. Linearizes the values between (x0,y0) and (x1,y1). Using this linearization, aplyes x and returns y
             * @param x0 value to linearize
             * @param x1 value to linearize
             * @param y0 value to linearize
             * @param y1 value to linearize
             * @return corresponding value of y=f(x) taking into account (x0,y0) and (x1,y1).
             */
            double LinSpace(double x0, double x1,
                            double y0, double y1,
                            double x);

    };  // class Swap

};  // namespace avoid


#endif // SWAP_SWAP_H_
