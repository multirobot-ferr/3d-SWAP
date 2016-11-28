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
#include <map>
#include <iostream>
#include <quick_mission/tinyxml2.h>

using namespace grvc::mbzirc;
using namespace grvc::utils;

using namespace std;
using namespace tinyxml2;

int main(int _argc, char** _argv)
{
	ArgumentParser args(_argc, _argv);

	// Open mission file
	string missionFile = args.getArgument("mission", string("mission.xml"));

	XMLDocument doc;
	doc.LoadFile(missionFile.c_str());
	map<int,Agent*>	robots;

	// Parse mission file
	{
		XMLNode* root = doc.FirstChild();
		if(!root) {
			cout << "Error loading xml mission file " << missionFile << "\n";
			return -1;
		}

		XMLElement* uav = root->FirstChildElement("uav");
		while(uav) {
			string uri;
			int id;
			uav->QueryIntAttribute("id", &id);

			const char* uriRaw = uav->Attribute("uri");
			if(uriRaw)
				uri = uriRaw;
			else {
				cout << "Error: uav " << id << " requires a valid uri attribute\n";
			}

			float x, y;
			uav->QueryFloatAttribute("x", &x);
			uav->QueryFloatAttribute("x", &y);

			// Create an agent using the specified informational
			robots.insert(make_pair(id, new Agent(uri, Vector2(x,y))));

			uav = uav->NextSiblingElement("uav");
		}
	}
	
	return 0;
}
