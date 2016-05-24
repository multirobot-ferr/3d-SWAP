#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>

//Our Action interface type
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;

class Gripper
{
	private:
		GripperClient * gripper_client_;
	
	public:
	//Action client initalization
		Gripper()
		{
			//Initialize the client for the Action interface to the gripper controller and tell the action client that we want to spin a thread by default
			gripper_client_ = new GripperClient("r_gripper_controller/gripper_action", true);

			//wait for the gripper action server to come up
			while(!gripper_client_->waitForServer(ros::Duration(5.0)))
			{
				ROS_INFO("Waiting for the r_gripper_controller/gripper_action server to come up");
			}
		}
		~Gripper()
		{
			delete gripper_client_;
		}
		
		//Open the Gripper
		void open()
		{
			pr2_controllers_msgs::Pr2GripperCommandGoal open;
			open.command.position = 0.08;
			open.command.max_effort = -1.0;

			ROS_INFO("Sending open goal");
			gripper_client_->sendGoal(open);
			gripper_client_->waitForResult();
			if(gripper_client_->getState() == action::SimpleClientGoalState::SUCCEEDED)
			{
				ROS_INFO("The Gripper opened!");
			}
			else
			{
				ROS_INFO("The Gripper failed to open");
			}
		}
		
		//Close the Gripper
		void close()
		{
			pr2_controllers_msgs__Pr2GripperCommandGoal squeeze;
			squeeze.command.position = 0.0;
			squeeze.command.max_effort = 50.0;
			
			ROS_INFO("Sending squeeze goal")
			gripper_client_->sendGoal(squeeze);
			gripper_client_->waitForResult();
			if(gripper_client_->getState() == action::SimpleClientGoalState::SUCCEEDED)
			{
				ROS_INFO("The Gripper closed!");
			}
			else
			{
				ROS_INFO("The Gripper failed to close");
			}
		}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "simple_gripper");
			
	Gripper gripper;
			
	gripper.open();
	gripper.close();
			
	return 0;
	
}
