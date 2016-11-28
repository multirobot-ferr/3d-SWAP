//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
// 
// Copyright (c) 2016 Carmelo J. Fernández-Agüera Tortosa
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
//----------------------------------------------------------------------------------------------------------------------
// Strategy testing and simulation environment for mbzirc competition
//----------------------------------------------------------------------------------------------------------------------
#include <gcs_core/strategy/strategy.h>
#include <gcs_core/agent/agent.h>
#include <grvc_utils/argument_parser.h>
#include <string>
#include <vector>
#include <quick_mission/tinyxml2.h>

using namespace grvc::mbzirc;
using namespace grvc::utils;

using namespace std;

int main(int _argc, char** _argv)
{
	ArgumentParser args(_argc, _argv);

	// Open mission file
	string missionFile = args.getArgument("mission", string("mission.xml"));


	vector<Agent*>	robots;
	robots.push_back(new Agent("/quad1", Vector2(0.f, 0.f)));
	
	return 0;
}
