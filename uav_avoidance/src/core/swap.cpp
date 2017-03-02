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
 * @file    swap.cpp
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

#include <swap.h>

namespace avoid
{

    /***************************************************
     *         Methods of the Swap class               *
     * *************************************************/

//    /**
//     * Constructor of the class
//     */
//    Swap::Swap()
//    {
//        //ctor
//    }
//
//    /* Default destructor */
//    Swap::~Swap()
//    {
//        //dtor
//    }

    /**
     * IsReady returns if swap is well configurated or not. And if not, explains why.
     */
    bool Swap::IsReady(  std::string& why_not)
    {
        if ( !PolarObstacleDiagram::IsReady( why_not))
        {
            return false;
        }

        // The parameters that has to be tuned are listed in order of tunning.







//        if (goal_lateral_vision_ == 0.0)
//        {
//            why_not =  "The value of the \"Goal Lateral Vision\" is not set\n";
//            why_not += "The goal lateral vision is a parameter between 360 & 10 deg that makes the system ignore where the ";
//            why_not += "conflicts are placed unless they are infront of the robot in a field of view of that size.";
//            why_not += "This allows the system to rotate around a convex conflict (if is set at less than 90 deg) surrounding ";
//            why_not += "those objects in the same way that a bug algorithm will do.";
//            why_not += "Recommended value, 90 deg";
//            return false;
//        }

        return true;
    }


    /**
     * Set the position of the goal with respect to the robot (in polars)
     */
     void Swap::SetGoal(double distance, double angle)
     {
         if (distance < 0)
         {
             distance *= -1;
             angle += M_PI;
         }

         goal_dist_ = distance;
         goal_angle_= ScaleAngle(angle);
     }

     /**
      * Calculates a direction and a speed for collision avoidance.
      */
     void Swap::CollisionAvoidance( double& v_ref, double& yaw_ref )
     {
         // Searches conflicts
         GetConflicts( conflictive_angles_ );

         if ( conflictive_angles_.empty() )
         {
             // No conflicts around, behave as desired
             statusOri_ = FREE;
         }
         else
         {
             ConflictsManager();
         }

         // Commanding behaviours according to the current status of statusOri_
         switch (statusOri_)
         {
             case FREE:
                 // The robot can travel freely to the goal.
                 v_ref   = GetDesiredVelocity();
                 yaw_ref = goal_angle_;
                 break;
             case RENCONTRE:
                 // The robot should stop while turning in the specified avoidance direction.
                 v_ref   = 0.0;
                 yaw_ref = yaw_avoidance_;
                 break;
             case RENDEZVOUS:
                 // The robot should surround the obstacle.
                 v_ref   = v_avoidance_;
                 yaw_ref = yaw_avoidance_;
                 break;
             case BLOCKED:
                 // The robot should keep position and orientation until the situation changes.
                 v_ref   = 0.0;
                 yaw_ref = 0.0;
                 break;
         }
     }

     /**
      * Configures the Goal Lateral Vision parameter
      */
     void Swap::SetGoalLateralVision( double goal_lateral_vision)
     {
         if (goal_lateral_vision >= 10.0 && goal_lateral_vision < 360.0)
         {
             // Converting the goal lateral vision in a prepared version for our algorithm
             goal_lateral_vision_ = goal_lateral_vision*M_PI/ (2.0 * 180.0);
         }
     }

     /**
      * Returns the state of the state machine
      */
     unsigned Swap::GetMachineState( std::string& state_name )
     {
         switch (statusOri_)
         {
            case FREE:
             state_name = "\e[32mFree";
             break;

            case RENDEZVOUS:
             state_name = "\e[33mRendezvous";
             break;

            case RENCONTRE: //Requested to stop
             state_name = "\e[41mRencontre";
             break;

            case BLOCKED:
             state_name = "\e[36mBlocked";
             break;
         }
         state_name += "\x1b[49m";

         return unsigned(statusOri_);
     }

     /**
      * Returns the state of the state machine if there is a change since last call
      */
     unsigned Swap::GetMachineStateChanges( std::string& state_name )
     {
         static int return_tmp = -1;
         if (return_tmp == -1      ||    // First call
             return_tmp != statusOri_)   // Change since last call
         {
             return_tmp = GetMachineState( state_name );
             return unsigned(return_tmp);
         }
         else
         {
             state_name = "";
             return 0;
         }
     }

     /**
      * Utility function Manages the conflicts and if a solution exists fills the references
      */
     void Swap::ConflictsManager()
     {
            // Depending if the turning procedure is CLOCKWISE or COUNTERCLOCKWISE we use one of the following readings
            double conflict_left_phi  = 0.0;
            double conflict_right_phi = 0.0;

            if (ComputeForbiddenDirections( conflict_left_phi, conflict_right_phi))
            {   // There are scape directions available
                bool conflictIgnorable = false;

                if (statusOri_ == FREE)
                {
                    // If the system was previously in the FREE state, it can see the goal in any direction
                    conflictIgnorable = AreConflictsIgnorable(conflict_left_phi, conflict_right_phi, goal_angle_, M_PI);
                }
                else
                {
                    // If the system was not previously in the FREE state, it is necessary to impose a goal_lateral_vision_ (helps on the U-Shapes)
                    conflictIgnorable = AreConflictsIgnorable(conflict_left_phi, conflict_right_phi, goal_angle_, goal_lateral_vision_);
                }

                if (conflictIgnorable)
                {
                    statusOri_ = FREE;
                }
                else
                {
                    // Computes orientations and velocities that allows the robot to surround the obstacle
                    // Deals with RENDESVOUZ/RENCONTRE state as well as CLOCKWISE, COUNTERCLOCKWISE and TOGOAL behaviours.
                    ApplyAvoidancePolicy(conflict_left_phi, conflict_right_phi);
                }
            }
            else   // Conflicts detected but no solution available
            {
                statusOri_ = BLOCKED;
            }
     }

     /**
      * Computes all the forbidden direction due to the inflated safety regions
      */
     bool Swap::ComputeForbiddenDirections( double& conflict_left_phi, double& conflict_right_phi)
     {
         if (conflictive_angles_.size() == 1)
         {  // There is only one conflict around
             conflict_left_phi  = conflictive_angles_[0];
             conflict_right_phi = conflictive_angles_[0];
             return true;
         }

         // more than 1 conflict

         /*
         Evaluate if the angle between conflicts is larger than pi, is the same that inflate each to [conflict.angle-pi/2 , conflict.angle+pi/2]
         and check if exist an space in between.

         If the angle between conflicts is larger than pi, it means that there is a navigable area on this region.

         Note: AngleDifference returns negative values if the difference is larger than pi. this happens because the
         error is shorter if you measure it counter-clockwise instead of clockwise.
         */

         for (int id_phi = 0; id_phi < conflictive_angles_.size(); ++id_phi)
         {
             int id_phi_minus = CircularIndex(id_phi -1, conflictive_angles_.size());
             if ( AngleDiff(conflictive_angles_[id_phi], conflictive_angles_[id_phi_minus]) < 0)
             {
                 conflict_left_phi   = conflictive_angles_[id_phi_minus];
                 conflict_right_phi  = conflictive_angles_[id_phi];
                 return true;
             }
         }

         return false;
     }


     /**
      * Utility function. Checks if all found conflicts can be ignored.
      */
     bool Swap::AreConflictsIgnorable(const double conflict_left_id_phi,
                                      const double conflict_right_id_phi,
                                      const double goal_angle,
                                      const double goal_lateral_vision_)
     {
         double left_angle = conflict_left_id_phi;
         double right_angle= conflict_right_id_phi;
         if ( fabs( AngleDiff(0, left_angle) ) < M_PI_2 ||
              fabs( AngleDiff(0, right_angle)) < M_PI_2   )
         {   // The robot is facing the non-navigable area, it will crash when continuing motion into the currenct direction.
             return false;
         }

         if (fabs(goal_angle) <= goal_lateral_vision_)
         {
            // The goal is visible from the robot's perspective
            if (fabs( AngleDiff(goal_angle, left_angle) ) < M_PI_2 ||
                fabs( AngleDiff(goal_angle, right_angle)) < M_PI_2)
            {
                // The goal belongs to the not navigable area
                return false;
            }
         }
         else
         {
             // The goal is not visible from the robot's perspective
             return false;
         }

         return true;
     }


     /**
      * Computes for the RENDEZVOUS or RENCONTRE states the necessary avoidance directions
      */
     void Swap::ApplyAvoidancePolicy( const double conflict_left_phi,
                                      const double conflict_right_phi)
     {
         // Computes the avoidance direction that allows the robot to surround the obstacle.
         // Relative to the current orientation! Positive value -> robot should steer left.

         double conflict2avoid_phi;

         switch (rot_behaviour_)
         {
             case CLOCKWISE:
                 yaw_avoidance_ = +M_PI_2;
                 conflict2avoid_phi = conflict_left_phi;
                 rot_ctrl_P_      = -fabs(rot_ctrl_P_);
                 break;

             case COUNTERCLOCKWISE:
                 yaw_avoidance_ = -M_PI_2;
                 conflict2avoid_phi = conflict_right_phi;
                 rot_ctrl_P_      = +fabs(rot_ctrl_P_);
                 break;
         }

         yaw_avoidance_ += conflict2avoid_phi;

         // The system tries to keep a certain rotation distance.
         yaw_avoidance_ += rot_ctrl_P_ * GetYawAvoidanceDistanceError( conflict2avoid_phi );

         // Depending on how far the system is from the desired orientation, the robot moves
         // slower or faster.
         /* As close the robot is to the desired distance, as faster can it go.

           1.0*lin_v_rendezvous_  +     +
                                 |    /   \
                                 |   /     \
           0.0*lin_v_rendezvous_  +-+---+---+--> errOri
                             -yaw_max_err_  0   yaw_max_err_
           */

         double pond_ori = LinSpace(0.0, yaw_max_err_, 1.0, 0.0, fabs(yaw_avoidance_));   // Value between 0.0 and 0.4
         v_avoidance_ = pond_ori*lin_v_rendezvous_;

         statusOri_ = (fabs(yaw_avoidance_) < yaw_max_err_) ? RENDEZVOUS : RENCONTRE;
     }

     /**
      * Returns the speed that the robot should have according to the situation
      */
     double Swap::GetDesiredVelocity()
     {
        /* As close the angle is to the desiredAngle, as faster we can go.

               1.0  +          +
                    |         / \
                    |       /     \
                    |     /         \
                    |   /             \
               0.0  +--+-------+-------+----->
                       -PI/2   0    PI/2
         */

        double pond_angle = std::max( 0.0, LinSpace(0, M_PI_2, 1.0, 0.0, fabs(goal_angle_)) );  // Value between 0 and 1

        /* As close the robot is to the goal, as slower we should go

                1.0   +          +------------
                      |         /
                      |       /
                0.1   |     +
                      |     |
                0.0   +-----+-----+------------>
                         d_goal_  d_approach_
         */

        double pond_d_goal = 1.0;

        if (goal_dist_ < d_goal_)
        {
            pond_d_goal = 0.0;
        }
        else if (goal_dist_ < d_approach_)
        {
            pond_d_goal = LinSpace(d_goal_, d_approach_, 0.1, 1.0, goal_dist_);
        }
        else
        {
            pond_d_goal = 1.0;
        }

        return pond_angle * pond_d_goal;
     }


     /**
      * Utility function. Computes the difference between two orientation angles
      */
     double Swap::AngleDiff(double ang1, double ang2)
     {
         // translate both angles to (-2*pi, +2*pi) range
         ang1 = ScaleAngle(ang1);
         ang2 = ScaleAngle(ang2);

         // duplicate first angle 2 times
         double angleRep[] = {ang1-2*M_PI, ang1, ang1+2*M_PI};

         // calculates the 3 errors and gets the minimum
         double diff = 2*M_PI;
         double diffAux;
         for(unsigned int i = 0; i < 3; i++)
         {
             diffAux = angleRep[i] - ang2;
             if (fabs(diffAux) < fabs(diff))
             {
                 diff = diffAux;
             }
         }
         return diff;
     }

     /**
      * Utility function. Linearizes the values between (x0,y0) and (x1,y1). Using this linearization, aplyes x and returns y
      */
     double Swap::LinSpace(double x0, double x1,
                           double y0, double y1,
                           double x)
     {
         /*
          *   y = a*x + b
          *
          *   y0 = a*x0 + b
          *   y1 = a*x1 + b
          */
         double y = std::numeric_limits<double>::max();;

         if (x0 != x1)
         {
             double a = (y0 - y1)/(x0 - x1);
             double b = y0 - a*x0;

             y = a*x + b;
         }

         return y;
     }


};  // namespace avoid
