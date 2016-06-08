//------------------------------------------------------------------------------
// GRVC MBZIRC Vision
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

#ifndef MBZIRC_CANDIDATE_H_
#define MBZIRC_CANDIDATE_H_

#include <Eigen/Eigen>

namespace mbzirc{
    struct Candidate{
        Eigen::Vector3d location;
        Eigen::Vector3d speed;

        Eigen::Matrix3d locationCovariance;
        Eigen::Matrix3d speedCovariance;

        /// Candidate's color. -1:unknown, 0: Black, 1: Red and 2:Blue. 666
        int color;

        /// Candidate's shape.
        int shape;

    };
    // Serialize candidate.
    std::ostream& operator<<(std::ostream& _os, const Candidate& _candidate);
    // Deserialize candidate.
    std::istream& operator>>(std::istream& _is, Candidate &_candidate);

}

#endif
