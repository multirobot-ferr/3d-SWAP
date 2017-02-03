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


#include <mbzirc_scheduler/Candidate.h>

#include <boost/tokenizer.hpp>
#include <vector>

using namespace std;
using namespace boost;

namespace mbzirc{
    //---------------------------------------------------------------------------------------------------------------------
    ostream &operator<<(ostream &_os, const Candidate &_candidate) {
        // Serialize location
            _os << _candidate.location[0] << ", " << _candidate.location[1] << ", " << _candidate.location[2] << endl;
        // Serialize speed
            _os << _candidate.speed[0] << ", " << _candidate.speed[1] << ", " << _candidate.speed[2] << endl;
        // Serialize covariance of location
            _os <<  _candidate.locationCovariance(0,0) << ", " << _candidate.locationCovariance(0,1) << ", " << _candidate.locationCovariance(0,2) << ", " <<
                    _candidate.locationCovariance(1,0) << ", " << _candidate.locationCovariance(1,1) << ", " << _candidate.locationCovariance(1,2) << ", " <<
                    _candidate.locationCovariance(2,0) << ", " << _candidate.locationCovariance(2,1) << ", " << _candidate.locationCovariance(2,2) << endl;
        // Serialize covariance of speed
            _os <<  _candidate.speedCovariance(0,0) << ", " << _candidate.speedCovariance(0,1) << ", " << _candidate.speedCovariance(0,2) << ", " <<
                    _candidate.speedCovariance(1,0) << ", " << _candidate.speedCovariance(1,1) << ", " << _candidate.speedCovariance(1,2) << ", " <<
                    _candidate.speedCovariance(2,0) << ", " << _candidate.speedCovariance(2,1) << ", " << _candidate.speedCovariance(2,2) <<  endl;
        // Serialize color
            _os << _candidate.color << std::endl;
        // Serialize shape
            _os << _candidate.shape << std::endl;
        // Serialize Size and angle
            _os << _candidate.width << ", " << _candidate.height << ", " << _candidate.rotation << std::endl;

        return _os;
    }

    //---------------------------------------------------------------------------------------------------------------------
    istream &operator>>(istream &_is, Candidate &_candidate) {
        auto parseNextLine = [](istream &_is){
            string lineContainer;
            vector<double> parsedNumbers;
            getline(_is,lineContainer);
            char_separator<char> separator(", ");
            tokenizer<char_separator<char>> tok(lineContainer, separator);
            for_each (tok.begin(), tok.end(), [&parsedNumbers](const string & _s) {  parsedNumbers.push_back(atof(_s.c_str())); } );
            return parsedNumbers;
        };

        // Deserialize location
        auto numbers = parseNextLine(_is);
        _candidate.location << numbers[0], numbers[1], numbers[2];

        // Deserialize speed
        numbers = parseNextLine(_is);
        _candidate.speed << numbers[0], numbers[1], numbers[2];

        // Deserialize covariance of location
        numbers = parseNextLine(_is);
        _candidate.locationCovariance << numbers[0], numbers[1], numbers[2], numbers[3], numbers[4], numbers[5], numbers[6],  numbers[7], numbers[8];

        // Deserialize covariance of speed
        numbers = parseNextLine(_is);
        _candidate.speedCovariance << numbers[0], numbers[1], numbers[2], numbers[3], numbers[4], numbers[5], numbers[6],  numbers[7], numbers[8];

        // Deserialize color
        numbers = parseNextLine(_is);
        _candidate.color = int(numbers[0]);

        // Deserialize shape
        numbers = parseNextLine(_is);
        _candidate.shape = int(numbers[0]);

        // Deseriaize size and orientation
        numbers = parseNextLine(_is);
        _candidate.width = numbers[0];
        _candidate.height = numbers[1];
        _candidate.rotation = numbers[2];

        return _is;
    }



}
