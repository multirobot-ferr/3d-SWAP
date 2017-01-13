/*
 * File Generated World from Gazebo
 * Author: Luis Manuel Ramirez de la Cova
 * Date: March 2016
 * Modificate: 26-04-2016
 * Organization: University of Seville, GRVC
 * Description: Algorithm that can generate a file .world
 */
 
#include <fstream> 
#include <iostream>
#include <algorithm>
#include <vector>
#include <ctime>
#include <cstdlib>
#include <stdio.h>
#include <stdlib.h>



using namespace std;

//Random Generator Function
int myrandom (int i)
{
	return std::rand()%i;
}

int main (int argc, char **argv)
{

	std::srand ( unsigned ( std::time(0) ) );
  	std::vector<float> myvector_x;
	std::vector<float> myvector_y;
	
	
	// Set values from x:
  	for (float i=-49; i<49; ++i) myvector_x.push_back(i); 

  	// Using built-in random generator:
  	std::random_shuffle ( myvector_x.begin(), myvector_x.end() );

 	// Using myrandom:
  	std::random_shuffle ( myvector_x.begin(), myvector_x.end(), myrandom);
  	
  	
  	  	
	  
	// Set values from y:
  	for (float i=-29; i<29; ++i) myvector_y.push_back(i); 

  	// using built-in random generator:
  	std::random_shuffle ( myvector_y.begin(), myvector_y.end() );

 	// using myrandom:
  	std::random_shuffle ( myvector_y.begin(), myvector_y.end(), myrandom);

 	/*std::cout << "myvector contains:";
	  for (std::vector<float>::iterator it=myvector_y.begin(); it!=myvector_y.end(); ++it)
    std::cout << ' ' << *it;

  	std::cout << '\n'; */
	
    string package = argv[1];
    string worldfile = package + "/world/generate_world.world";
    string meshfile = package + "/model/stage_01_gazebo.dae";
	ofstream myfile;

    myfile.open (worldfile);
	myfile <<"<?xml version=\"1.0\" ?>\n"
		   <<"<sdf version=\"1.4\">\n"
  		   <<"	<world name=\"default\">\n"
		   <<"\n"
           <<"		<!-- A global light source -->\n"
           <<"		<include>\n"
           <<"			<uri>model://sun</uri>\n"
    	   <<"		</include>\n"
		   <<"\n"
    	   <<"		<!-- A ground plane -->\n"
           <<"		<include>\n"
      	   <<"			<uri>model://ground_plane</uri>\n"
    	   <<"		</include>\n"
    	   <<"\n"
           <<"\n"
		   <<"		<!--Model Blender of Stage-->\n"
		   <<"\n"
	       <<"		<model name=\"testbed_scene\">\n"
		   <<"			<static>true</static>\n"
		   <<"			<pose>23.158678 75.310852 0 0 0 1.57</pose>\n"
		   <<"			<link name=\"test_scene\">\n"
		   <<"				<visual name=\"visual\">\n"
		   <<"					<geometry>\n"
           <<"						<mesh><uri>"<<meshfile<<"</uri></mesh>\n"
		   <<"					</geometry>\n"
		   <<"				</visual>\n"
		   <<"			</link>\n"
		   <<"		</model>\n"
		   <<"\n"
		   <<"		<!-- Automatic Random Position Models -->\n"
    	   <<"		<!--*********************************-->\n"
           <<"\n"
          <<"			<model name=\"Dropping_box\">\n"
          <<"				<pose>-66 25 0.01 0 0 0</pose>\n"
          <<"				<include>\n"
          <<"					<uri>model://dropping_box</uri>\n"
          <<"				</include>\n"
          <<"			</model>\n"
          <<"\n"
    	   <<"			<model name=\"Object_Black_Cylinder1\">\n"
    	   <<"				<pose>"
           << 						myvector_x.at(0) << " " <<  myvector_y.at(0) <<" 0.05 0 0 0"
    	   <<"				</pose>\n"
           <<"				<include>\n"
           <<"					<uri>model://object_black_cylinder_mbzirc</uri>\n"
           <<"				</include>\n"
      	   <<"			</model>\n"
      	   <<"\n"
      	   <<"			<model name=\"Object_Black_Cylinder2\">\n"
    	   <<"				<pose>"
           << 						myvector_x.at(1) << " " <<  myvector_y.at(1) <<" 0.05 0 0 0"
    	   <<"				</pose>\n"
           <<"				<include>\n"
           <<"					<uri>model://object_black_cylinder_mbzirc</uri>\n"
           <<"				</include>\n"
      	   <<"			</model>\n"
      	   <<"\n"
      	   <<"			<model name=\"Object_Black_Box1\">\n"
    	   <<"				<pose>"
           << 						myvector_x.at(2) << " " <<  myvector_y.at(2) <<" 0.05 0 0 0"
    	   <<"				</pose>\n"
           <<"				<include>\n"
           <<"					<uri>model://object_black_box_mbzirc</uri>\n"
           <<"				</include>\n"
      	   <<"			</model>\n"
      	   <<"\n"
      	   <<"			<model name=\"Object_Blue_Cylinder1\">\n"
    	   <<"				<pose>"
           << 						myvector_x.at(3) << " " <<  myvector_y.at(3) <<" 0.05 0 0 0"
    	   <<"				</pose>\n"
           <<"				<include>\n"
           <<"					<uri>model://object_blue_cylinder_mbzirc</uri>\n"
           <<"				</include>\n"
      	   <<"			</model>\n"
      	   <<"\n"
      	   <<"			<model name=\"Object_Blue_Cylinder2\">\n"
    	   <<"				<pose>"
           << 						myvector_x.at(4) << " " <<  myvector_y.at(4) <<" 0.05 0 0 0"
    	   <<"				</pose>\n"
           <<"				<include>\n"
           <<"					<uri>model://object_blue_cylinder_mbzirc</uri>\n"
           <<"				</include>\n"
      	   <<"			</model>\n"
      	   <<"\n"
      	   <<"			<model name=\"Object_Blue_Box1\">\n"
    	   <<"				<pose>"
           << 						myvector_x.at(5) << " " <<  myvector_y.at(5) <<" 0.05 0 0 0"
    	   <<"				</pose>\n"
           <<"				<include>\n"
           <<"					<uri>model://object_blue_box_mbzirc</uri>\n"
           <<"				</include>\n"
      	   <<"			</model>\n"
      	   <<"\n"
      	   <<"			<model name=\"Object_Blue_Box2\">\n"
    	   <<"				<pose>"
           << 						myvector_x.at(6) << " " <<  myvector_y.at(6) <<" 0.05 0 0 0"
    	   <<"				</pose>\n"
           <<"				<include>\n"
           <<"					<uri>model://object_blue_box_mbzirc</uri>\n"
           <<"				</include>\n"
      	   <<"			</model>\n"
      	   <<"\n"
      	   <<"			<model name=\"Object_Red_Box1\">\n"
    	   <<"				<pose>"
           << 						myvector_x.at(7) << " " <<  myvector_y.at(7) <<" 0.05 0 0 0"
    	   <<"				</pose>\n"
           <<"				<include>\n"
           <<"					<uri>model://object_red_box_mbzirc</uri>\n"
           <<"				</include>\n"
      	   <<"			</model>\n"
      	   <<"\n"
      	   <<"			<model name=\"Object_Red_Cylinder1\">\n"
    	   <<"				<pose>"
           << 						myvector_x.at(8) << " " <<  myvector_y.at(8) <<" 0.05 0 0 0"
    	   <<"				</pose>\n"
           <<"				<include>\n"
           <<"					<uri>model://object_red_cylinder_mbzirc</uri>\n"
           <<"				</include>\n"
      	   <<"			</model>\n"
      	   <<"\n"
      	   <<"			<model name=\"Object_Red_Cylinder2\">\n"
    	   <<"				<pose>"
           << 						myvector_x.at(9) << " " <<  myvector_y.at(9) <<" 0.05 0 0 0"
    	   <<"				</pose>\n"
           <<"				<include>\n"
           <<"					<uri>model://object_red_cylinder_mbzirc</uri>\n"
           <<"				</include>\n"
      	   <<"			</model>\n"
      	   <<"\n"
      	   <<"			<model name=\"Object_Large_Red\">\n"
    	   <<"				<pose>"
           << 						myvector_x.at(10) << " " <<  myvector_y.at(10) <<" 0.05 0 0 0"
    	   <<"				</pose>\n"
           <<"				<include>\n"
           <<"					<uri>model://object_large_red_mbzirc</uri>\n"
           <<"				</include>\n"
      	   <<"			</model>\n"
      	   <<"\n"
      	   <<"			<model name=\"Object_Large_Blue\">\n"
    	   <<"				<pose>"
           << 						myvector_x.at(11) << " " <<  myvector_y.at(11) <<" 0.05 0 0 0"
    	   <<"</pose>\n"
           <<"				<include>\n"
           <<"					<uri>model://object_large_blue_mbzirc</uri>\n"
           <<"				</include>\n"
      	   <<"			</model>\n"
      	   <<"\n"
      	   <<"			<model name=\"Object_Large_Black\">\n"
    	   <<"				<pose>"
           << 						myvector_x.at(12) << " " <<  myvector_y.at(12) <<" 0.05 0 0 0"
    	   <<"				</pose>\n"
           <<"				<include>\n"
           <<"					<uri>model://object_large_black_mbzirc</uri>\n"
           <<"				</include>\n"
      	   <<"			</model>\n"
      	   <<"	</world>\n"
      	   <<"</sdf>\n"

	;
	myfile.close();
	
	
	return 0;
}
