/*
 *   Trajectory Generation Node for mobile object in Arena MBZIRC
 *   Author: Luis Manuel Ramírez de la Cova
 *   Date: March 2016
 *   
 *   Description: This executable generates the trajectory defined in terms of the waypoints contained in the file .txt
 */



//Standard library
#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <string.h>
#include <signal.h>
#include <stdlib.h>
#include <pthread.h>

//ROS library
#include <ros/ros.h>
#include "std_msgs/String.h"

#define POSITION_ERROR_LIMIT 0.25F    //Error threshold for jumpling between waypoints

//Structures
struct wayPoint
{
	double xref;
	double yref;
	double zref;
};


//Namespaces
using namespace std;


//Function declaration
void GoToPosition(const ros::Publisher & trGenPublisher, double x, double y, double z);
void quit(int sig);

//Thread declaration
static void* keyboardThreadFuntion(void * args);


//Global variable declaration
string nodeName;
int wayPointTrackingState = 1;			//1: Approaching to goal; 2: Leaving goal; 3: trajectory interrupted
bool firstWayPoint = true;
double xstop = 0;
double ystop = 0;
double zstop = 0;


int main(int argc, char** argv)
{
	string topicName;
	string line;
	ifstream trajectoryFile;
	ros::Publisher trGenPublisher;
	vector<struct wayPoint> wayPointVector;
	struct wayPoint wayPointAux;
	unsigned int wayPointIndex = 0;
	pthread_t keyboardThread;
	
	cout << endl;
	cout << "Trajectory Generation Node for Mobile Object" << endl;
	cout << "Author: Luis Manuel Ramirez de la Cova" << endl;
	cout << "Date: March 2016" << endl;
	cout << "Organization: University of Seville, GRVC" << endl;
	cout << "------------------------------------------" << endl << endl;
	
	signal(SIGINT, quit);
	
	//Try to open trajectory file
	trajectoryFile.open(argv[2]);
	if(trajectoryFile.is_open() == false)
	{
		cout << endl << "ERROR: Could not open trajectory file" << endl;
		return -1;
	}
	else
	{
		//Init ROS
		nodeName = "TrajectoryGenNode";
		nodeName.append(argv[1]);
		ros::init(argc, argv, nodeName.c_str());
		
		//Create the node Handle
		ros::NodeHandle n;
		
		//Set the reference speed
		control_ref_to_send.c_reference_rw.cruise = 0.4;
		
		
		//Create thread for reading keyboards inputs
		if(pthread_create(&keyboardThread, NULL, &keyboardThreadFunction, (void*)NULL))
			cout << "ERROR: could not create keyboard thread!" <<endl:
			
		//Load trajectory as a vector of way points
		while(getline(trajectoryFile, line))
		{
			//Check if the line is not a comment and if it has propper length
			if(strncmp(line.c_str(), "//", 2) !=0 && line.length() >= 9)
			{
				// Get X-position
				string xPosStr = line.substr(0, line.find("\t"));
				line = line.substr(line.find("\t") + 1);
				// Get Y-position
				string yPosStr = line.substr(0, line.find("\t"));
				line = line.substr(line.find("\t") + 1);
				// Get Z-position
				string zPosStr = line.substr(0, line.find("\t"));
				line = line.substr(line.find("\t") + 1);
				// Get yaw
				string yawPosStr = line.substr(0, line.find("\t"));
				line = line.substr(line.find("\t") + 1);
				// Get delay
				string delayStr = line;
				
				//Convert string values to double
				wayPointAux.xref = atof(xPosStr.c_str());
				wayPointAux.yref = atof(yPosStr.c_str());
				wayPointAux.zref = atof(zPosStr.c_str());
				wayPointAux.yawref = atof(yawPosStr.c_str());
			
				//Add the way point to the vector
				wayPointVector.push_back(wayPointAux);
			}
		}
	}
}






















