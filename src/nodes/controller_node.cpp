#include "ros/ros.h"
#include "std_msgs/String.h"
#include "controller.h"


#include <sstream>
std::string controller_name;

namespace rrc_control{
	class Controller_Node{
		Controller_Node(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
		~Controller_Node();
		double value;

		ros::NodeHandle nh_;
		ros::NodeHandle private_nh_;
		void InitializeParams();
	};


	Controller_Node::Controller_Node(
		const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
			:nh_(nh), private_nh_(private_nh) {
		InitializeParams();
	}

	Controller_Node::~Controller_Node(){}

	void Controller_Node::InitializeParams(){
	}
}





int main(int argc, char *argv[]){
	ros::init(argc, argv, "controller_node");
	ros::NodeHandle nh("~");
	nh.getParam("controller_name", controller_name);
	ROS_INFO_STREAM("Controller: " << controller_name);

	ros::spin();
	return 0;
}