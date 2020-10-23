#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Eigen>
#include <thread>
#include <chrono>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/conversions.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PointStamped.h"
#include <std_srvs/Empty.h>


int main(int argc, char **argv)

{

ros::init(argc, argv, "traj_gen");

ros::NodeHandle n;
ros::NodeHandle nh_private("~");
// ros::Publisher trajectory_pub =
//       n.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
//           mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
// ROS_INFO_STREAM("Default")
ros::Publisher traj_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/pelican/command/trajectory", 20);
// ros::Publisher vis_pub = n.advertise<visualization_msgs::MarkerArray>( "visualization_marker_snap", 100 );

ros::Rate sleep_rate(100);


std_srvs::Empty srv;
bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
unsigned int i = 0;

// Trying to unpause Gazebo for 10 seconds.
while (i <= 10 && !unpaused) {
ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
std::this_thread::sleep_for(std::chrono::seconds(1));
unpaused = ros::service::call("/gazebo/unpause_physics", srv);
++i;
}

if (!unpaused) {
ROS_FATAL("Could not wake up Gazebo.");
return -1;
} else {
ROS_INFO("Unpaused the Gazebo simulation.");
}

ros::Duration(5.0).sleep();

/*
while(ros::ok())
  {
*/    mav_trajectory_generation::Vertex::Vector vertices;
    const int dimension = 3;
    const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
    mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);

    start.makeStartOrEnd(Eigen::Vector3d(0, 0, 0), derivative_to_optimize);
    vertices.push_back(start);

    middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-0.5, -0.5, 0.5));
    vertices.push_back(middle);


    end.makeStartOrEnd(Eigen::Vector3d(-1, -1, 1), derivative_to_optimize);
    vertices.push_back(end);


    std::vector<double> segment_times;
    const double v_max = 2;
    const double a_max = 2;
    segment_times = estimateSegmentTimes(vertices, v_max, a_max);

    const int N = 10;
    mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
    opt.solveLinear();

    mav_trajectory_generation::Segment::Vector segments;
    opt.getSegments(&segments);

    mav_trajectory_generation::Trajectory trajectory;
    opt.getTrajectory(&trajectory);


    // Splitting:
    // mav_trajectory_generation::Trajectory x_trajectory = trajectory.getTrajectoryWithSingleDimension(1);

    // Compositing:
    // mav_trajectory_generation::Trajectory trajectory_with_yaw; trajectory.getTrajectoryWithAppendedDimension(yaw_trajectory, &trajectory_with_yaw);


    // Single sample:
    double sampling_time = 2.0;
    int derivative_order = mav_trajectory_generation::derivative_order::POSITION;
    Eigen::VectorXd sample = trajectory.evaluate(sampling_time, derivative_order);

    // Sample range:
    double t_start = 2.0;
    double t_end = 10.0;
    double dt = 0.01;
    std::vector<Eigen::VectorXd> result;
    std::vector<double> sampling_times; // Optional.
    trajectory.evaluateRange(t_start, t_end, dt, derivative_order, &result, &sampling_times);


    mav_msgs::EigenTrajectoryPoint state;
    mav_msgs::EigenTrajectoryPoint::Vector states;

    // Whole trajectory:
    double sampling_interval = 0.01;
    bool success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &states);

            
    int traj_size = states.size();        

    for(i=0; i<traj_size; i++)
    {
      trajectory_msgs::MultiDOFJointTrajectory traj_msg;
      mav_msgs::msgMultiDofJointTrajectoryFromEigen(states[i], &traj_msg);
      traj_msg.header.stamp = ros::Time::now();

      traj_pub.publish(traj_msg);
      sleep_rate.sleep();
    }
    
    vertices.clear();
    sleep_rate.sleep();
    

    ros::spinOnce();

}