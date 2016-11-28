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
#include <vector>
#include <iostream>
#include <quick_mission/tinyxml2.h>

using namespace grvc::mbzirc;
using namespace grvc::utils;

using namespace std;
using namespace tinyxml2;

struct Command {
	virtual void run() = 0;
	static Command* buildFromXml(XMLElement* node);
};

struct ParallelCommand {
	vector<Command*>	children;
};

struct SequenceCommand : Command {
	int uavId;
};

bool parseMissionFile(const char* missionFile, map<int,Agent*>& robots, vector<Command*> commands) {
	XMLDocument doc;
	doc.LoadFile(missionFile);
	
	XMLNode* root = doc.RootElement();
	if(!root) {
		cout << "Error loading xml mission file " << missionFile << "\n";
		return false;
	}

	// Get list of robots
	XMLElement* uav = root->FirstChildElement("uav");
	while(uav) {
		int id = uav->IntAttribute("id");

		const char* uriRaw = uav->Attribute("uri");
		string uri;
		if(uriRaw)
			uri = uriRaw;
		else {
			cout << "Error: uav " << id << " requires a valid uri attribute\n";
			return false;
		}

		float x = uav->FloatAttribute("x");
		float y = uav->FloatAttribute("y");

		// Create an agent using the specified informational
		robots.insert(make_pair(id, new Agent(uri, Vector2(x,y))));

		uav = uav->NextSiblingElement("uav");
	}

	// Iterate over mission commands
	XMLElement* commandNode = root->FirstChildElement("command");
	while(commandNode) {
		Command* cmd = Command::buildFromXml(commandNode);
		if(!cmd) {
			cout << "Error parsing command\n";
			return false;
		}
		commands.push_back(cmd);
		commandNode = commandNode->NextSiblingElement("command");
	}

	return true;
}

int main(int _argc, char** _argv)
{
	ArgumentParser args(_argc, _argv);

	// Read mission file
	string missionFile = args.getArgument("mission", string("mission.xml"));

	map<int,Agent*>	robots;
	vector<Command*>	commands;

	if(!parseMissionFile(missionFile.c_str(), robots, commands))
		return -1;

	// Run commands.
	// Agents need to be updated periodically, so we'll run a separate thread for them, and feed them the commands sequentially.
	
	return 0;
}
