//----------------------------------------------------------------------------------------------------------------------
// GRVC Utils
//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2016 GRVC University of Seville
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//----------------------------------------------------------------------------------------------------------------------

/**
 * @file pid_controller.h
 * @author Eduardo Ferrera
 * @version 2.0
 * @date    16/11/17
 * @short: PID controller with dynamic reconfigures on it 
 *
 * The following pid controller is capable of handling time and use dynamic_reconfigures 
 * for parameter tunning
 */

#ifndef HANDY_TOOLS_PID_CONTROLLER_H
#define HANDY_TOOLS_PID_CONTROLLER_H

#include <string>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <uav_avoidance/pidConfig.h>    

namespace grvc { namespace utils {



// TODO: Add dynamic tuning tools
class PidController {
    public:
        PidController(std::string _name = "pid", double _k_p = 1.0, double _k_i = 0.0, double _k_d = 0.0) {
            name_ = _name;
            k_p_ = _k_p;
            k_i_ = _k_i;
            k_d_ = _k_d;
        }

        double control_signal(double _error, double _dt) {
            // TODO: anti-windup and other pid sofistications :)
            error_sum_ += _error * _dt;
            error_sum_ = num_limit(error_sum_, integral_limit_);
            double error_diff = _error - previous_error_;
            previous_error_ = _error;

            double output = k_p_ * _error + k_i_ * error_sum_ + k_d_ * error_diff / _dt;
            
            output = num_limit(output, output_limit_);
            return output;
        }

        void reset()
        {
            error_sum_ = 0.0;
            previous_error_ = 0.0;
        }

        void set_k_p(double _k_p)
        {
            k_p_ = _k_p;
        }

        void set_k_i(double _k_i)
        {
            k_i_ = _k_i;
        }

        void set_k_d(double _k_d)
        {
            k_d_ = _k_d;
        }

        void set_integral_limit(double _integral_limit)
        {
            integral_limit_ = _integral_limit;
        }

        void set_output_limit(double _output_limit)
        {
            if (_output_limit > 0.0)
                output_limit_ = _output_limit;
        }

        const std::string get_name()
        {
            return name_;
        }

    private:
        std::string name_;
        double k_p_;
        double k_i_;
        double k_d_;
        double error_sum_      = 0.0;
        double previous_error_ = 0.0;
        double integral_limit_ = std::numeric_limits<double>::max();
        double output_limit_   = std::numeric_limits<double>::max();

        double num_limit(double num, double limit)
        {
            num = std::min(num, +limit);
            num = std::max(num, -limit);

            return num;
        }
};

class PidROS: public PidController
{
    public:
        PidROS(std::string _pid_name = "pid", 
                double _k_p = 1.0, double _k_i = 0.0, double _k_d = 0.0,
                double _output_limit = 2.0, double _integral_limit = 5.0):
                server(_pid_name)
        {
            pid_name_ = _pid_name;
            set_k_p(_k_p);
            set_k_i(_k_i);
            set_k_d(_k_d);
            set_output_limit(_output_limit);
            set_integral_limit(_integral_limit);

            f = boost::bind(&PidROS::ReconfigureCallback, this, _1, _2);
            server.setCallback(f);
        };

    double control_signal(double _error)
    {
        ros::Duration dt = ros::Time::now() - last_update_;
        last_update_ = ros::Time::now();
        
        if (!started_)
        {
            started_ = true;
            return 0.0;
        }
        else
        {
            //ROS_INFO("Control signal %f, error %f", PidController::control_signal(_error, dt.toSec()), _error);
            double ref = PidController::control_signal(_error, dt.toSec());
            if (!std::isnan(ref))
            {
                return ref;
            }
            else
            {
                ROS_ERROR("Error in %s pid. Sleeping time too short.", pid_name_.c_str());
                return 0.0;
            }
        }
    }

    private:
        // Time management
        bool started_ = false;
        ros::Time last_update_ = ros::Time::now();

        // Parameter management
        std::string pid_name_;
        double k_p_, k_i_, k_d_;

        dynamic_reconfigure::Server<uav_avoidance::pidConfig> server;
        dynamic_reconfigure::Server<uav_avoidance::pidConfig>::CallbackType f;

        void ReconfigureCallback(uav_avoidance::pidConfig &config, uint32_t level)
        {
            ROS_INFO("Reconfigure Request in pid %s: \n" 
                     "\t k_p=%f k_i=%f, k_d=%f\n"
                     "\t pid_output_limit=%f integral_limit=%f",
                    pid_name_.c_str(), 
                    config.k_p, config.k_i, config.k_d,
                    config.pid_output_limit, config.integral_limit);

            set_k_p(config.k_p);
            set_k_i(config.k_i);
            set_k_d(config.k_d);
            set_integral_limit(config.integral_limit);
            set_output_limit(config.pid_output_limit);            
        }
};

}} // namespace grvc::utils

#endif  // HANDY_TOOLS_PID_CONTROLLER_H
