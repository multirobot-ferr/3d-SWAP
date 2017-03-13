//------------------------------------------------------------------------------
// GRVC MBZIRC 
//------------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2017 GRVC University of Seville
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

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <gcs_state_machine.h>
#include <grvc_utils/frame_transform.h>
#include <DeployArea.h>

int main(int _argc, char **_argv) {
	ros::init(_argc, _argv, "GCS_STATE_MACHINE");
  
	// Read dropping area from xml
	tinyxml2::XMLDocument doc;
    std::string xml_file;
    ros::param::get("frames_file", xml_file);
    std::string path = ros::package::getPath("mbzirc_launchers") + xml_file;
			
    doc.LoadFile(path.c_str());
        
    tinyxml2::XMLNode* root = doc.RootElement();
    if(!root) {
        ROS_ERROR("Error loading xml file %s\n", xml_file.c_str());
        return -1;
	}

	// Get dropping game limits
	tinyxml2::XMLElement* dropping_game_limits_element = root->FirstChildElement("dropping_game_limits");
	tinyxml2::XMLElement* d_xmin_element = dropping_game_limits_element->FirstChildElement("x_min");
    double d_x_min;
	d_xmin_element->QueryDoubleText(&d_x_min);
	tinyxml2::XMLElement* d_xmax_element = dropping_game_limits_element->FirstChildElement("x_max");
    double d_x_max;
	d_xmax_element->QueryDoubleText(&d_x_max);
	tinyxml2::XMLElement* d_ymin_element = dropping_game_limits_element->FirstChildElement("y_min");
    double d_y_min;
	d_ymin_element->QueryDoubleText(&d_y_min);
	tinyxml2::XMLElement* d_ymax_element = dropping_game_limits_element->FirstChildElement("y_max");
    double d_y_max;
	d_ymax_element->QueryDoubleText(&d_y_max);
	
	double deploy_x = (d_x_min + d_x_max)/2.0;
	double deploy_y = (d_y_min + d_y_max)/2.0;

	float deploy_area_radius = 8.0;  // TODO: Test!

	grvc::utils::frame_transform frameTransform;
	geometry_msgs::Point deploy_point = frameTransform.game2map(grvc::utils::constructPoint(deploy_x, deploy_y, 3.0));  
	DeployAreaHandle deploy_area(deploy_point, deploy_area_radius);

	GcsStateMachine gcs;
	if(!gcs.init()){
		ROS_ERROR("ERROR INITIALIZING GCS!");
		return -1;	
	}
	
	ros::spin();
}
