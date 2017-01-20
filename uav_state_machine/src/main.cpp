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

#include <std_msgs/String.h>
#include <grvc_utils/argument_parser.h>

#include <thread>
#include <chrono>
#include <mutex>

#include <uav_state_machine/state_machine.h>

using namespace grvc;
using namespace std;

mbzirc::Candidate specs;
bool target_state = false;

mbzirc::Candidate bestCandidateMatch(const mbzirc::CandidateList &_list, const mbzirc::Candidate &_specs);

int main(int _argc, char** _argv){

    std::cout << "Setting up" << std::endl;

    // Init services.
    grvc::utils::ArgumentParser args(_argc, _argv);
    
    UavStateMachine uav_sm(args);
    while(true){

    }
}

mbzirc::Candidate bestCandidateMatch(const mbzirc::CandidateList &_list, const mbzirc::Candidate &_specs){
    double bestScore= 0;
    mbzirc::Candidate bestCandidate;
    bestCandidate.location = {999,9999,9999};
    for(auto&candidate:_list.candidates){
        double score = 0;
        if(candidate.color == _specs.color){
            score +=1;
        }

        if(candidate.shape == _specs.shape){
            score +=1;
        }

        if((candidate.location - _specs.location).norm() < (bestCandidate.location - _specs.location).norm()){
            score +=1;
        }

        if(score > bestScore){
            bestCandidate = candidate;
        }
    }
    return bestCandidate;
}
