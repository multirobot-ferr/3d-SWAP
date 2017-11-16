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
#ifndef HANDY_TOOLS_PID_CONTROLLER_H
#define HANDY_TOOLS_PID_CONTROLLER_H

#include <string>

// Dynamic reconfigure
#include <dynamic_reconfigure/Config.h>

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
        error_sum_ = num_limit(error_sum_, integer_limit_);
        double error_diff = _error - previous_error_;
        previous_error_ = _error;

        double output = k_p_ * _error + k_i_ * error_sum_ + k_d_ * error_diff / _dt;
        
        output = num_limit(output, output_limit_);
        return output;
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

    void set_integer_limit(double _integer_limit)
    {
        integer_limit_ = _integer_limit;
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
    double integer_limit_ = std::numeric_limits<double>::max();
    double output_limit_  = std::numeric_limits<double>::max();

    double num_limit(double num, double limit)
    {
        num = std::min(num, +limit);
        num = std::max(num, -limit);

        return num;
    }
};

}} // namespace grvc::utils

#endif  // HANDY_TOOLS_PID_CONTROLLER_H
