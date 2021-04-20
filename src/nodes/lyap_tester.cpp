
#include <fstream>
#include <iostream>

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <unsupported/Eigen/MatrixFunctions>
#include <rrc_control/math.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/UInt8.h>

bool sim_running = false;

static const int64_t kNanoSecondsInSecond = 1000000000;

void callback(const sensor_msgs::ImuPtr& msg) {
  sim_running = true;
}


class State{
public:
	State()
		: waiting_time_(0), state_(0){}
	State(double waiting_time, int state)
		: waiting_time_(waiting_time), state_(state){}
	double waiting_time_;
	uint state_;
};


int main(int argc, char** argv){
	ros::init(argc, argv, "state_switch_node");
	ros::NodeHandle nh;

	ros::V_string args;
	ros::removeROSArgs(argc, argv, args);

	if (args.size() != 2){
		ROS_INFO_STREAM("Usage: state_switch_node <file_name> \n The data should be structured as: waiting_time[s] state");
		return -1;
	}

	std::vector<State> states;
	std::ifstream state_file(args.at(1).c_str());

	double t;
	uint s;

	if (state_file.is_open()){
		while (state_file >> t >> s){
			states.push_back(State(t,s));
		}
		state_file.close();
		ROS_INFO_STREAM(states.size() << " states read.");
	}

	else{
		ROS_INFO_STREAM("Unable to open the file.");
		return -1;
	}

	ros::Publisher state_pub = nh.advertise<std_msgs::UInt8>("/switch_state", 1000);


	// The IMU is used, to determine if the simulator is running or not.
	ros::Subscriber sub = nh.subscribe("imu", 10, &callback);

	ROS_INFO("Wait for simulation to become ready...");

	// while (!sim_running && ros::ok()) {
	// ros::spinOnce();
	// ros::Duration(0.1).sleep();
	// }

	ROS_INFO("Ok");
	std_msgs::UInt8 state_msg;
	ros::Rate loop_rate(10);

	for (int i=0; i<states.size(); i++) {
		ros::Duration(states[i].waiting_time_).sleep();
		state_msg.data = states[i].state_;
		ROS_INFO_STREAM(state_msg);
		state_pub.publish(state_msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	ros::Duration(2).sleep();

	ROS_INFO_STREAM("State switch node is shutting down");

	ros::spin();
}