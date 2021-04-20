#include <ros/ros.h>
#include <std_msgs/String.h> 
#include <stdio.h>
#include <math.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Float64.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/Thrust.h>


//#include </home/mahesh/catkin_ws/src/beginner_tutorials/src/Qualisys.h>

static bool armed = false;
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "pub_setpoints");
  ros::NodeHandle n;

  ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>
          ("mavros/state", 10, state_cb);
  ros::ServiceClient arming_client = n.serviceClient<mavros_msgs::CommandBool>
          ("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = n.serviceClient<mavros_msgs::SetMode>
          ("mavros/set_mode");
  ros::Publisher pub_att = n.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_attitude/cmd_vel",100);
  ros::Publisher pub_thr = n.advertise<mavros_msgs::Thrust>("/mavros/setpoint_attitude/thrust", 100);
  ros::Publisher angularVelPub_ = n.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);

  ros::Rate loop_rate(100); 
  geometry_msgs::TwistStamped cmd_att;
  mavros_msgs::Thrust cmd_thr;
  int count = 1;
  double v[3]={1.0, 0.0, 0.0};
  double lambda = 1.0;


  double v_norm=sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
  double theta=0.0;

  //the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);


  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  ros::Time last_request = ros::Time::now();

  // wait for FCU connection
  while(ros::ok() && !current_state.connected){
      ros::spinOnce();
      rate.sleep();
  }


    while(ros::ok()){
        if(!armed){
          if( current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent){
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
          }
          else {
                if( !current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))){
                    if( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success){
                        ROS_INFO("Vehicle armed");
                        // armed = true;
                        break;
                    }
                    last_request = ros::Time::now();
                }
            }
        }
    }
    
 
        //PositionReciever qp;
        //Body some_object;
        //qp.connect_to_server();
 
     
  while(ros::ok()){
     //some_object = qp.getStatus();
      // some_object.print();
      //printf("%f\n",some_object.position_x);
      //Create attitude command message
     ros::Time time_now = ros::Time::now();
     cmd_att.header.stamp = time_now;
     cmd_att.header.seq=count;
     cmd_att.header.frame_id = 1;
     // cmd_att.pose.position.x = 0.0;//0.001*some_object.position_x;
     // cmd_att.pose.position.y = 0.0;//0.001*some_object.position_y;
     // cmd_att.pose.position.z = -1.0;//0.001*some_object.position_z;

     // cmd_att.pose.orientation.x = sin(theta/2.0)*v[0]/v_norm;
     // cmd_att.pose.orientation.y = sin(theta/2.0)*v[1]/v_norm;
     // cmd_att.pose.orientation.z = sin(theta/2.0)*v[2]/v_norm;
     // cmd_att.pose.orientation.w = cos(theta/2.0);

     cmd_att.twist.angular.x = 0;
     cmd_att.twist.angular.y = 0;
     cmd_att.twist.angular.z = 1;
    /*
     double q_norm = sqrt(sin(theta/2.0)*sin(theta/2.0)+cos(theta/2.0)*cos(theta/2.0));
      printf("%f\t",v_norm);
      printf("%f\n", q_norm);
  */
     // ROS_INFO("Publish");
     //Create throttle command message
     cmd_thr.thrust = lambda;
     cmd_thr.header.stamp = time_now;
     cmd_thr.header.seq=count;
     cmd_thr.header.frame_id = 1;
     
     pub_att.publish(cmd_att);
     pub_thr.publish(cmd_thr);
     mavros_msgs::AttitudeTarget raw_cmd;
     raw_cmd.header.stamp = ros::Time::now();
     raw_cmd.header.frame_id = "map";
     raw_cmd.body_rate.x = 0;
     raw_cmd.body_rate.y = 0;
     raw_cmd.body_rate.z = 0;
     raw_cmd.thrust = 0.8;
     raw_cmd.type_mask = 128;
     // angularVelPub_.publish(raw_cmd);

     ros::spinOnce();
     count++;
     // theta=0.3*sin(count/300.0);
     loop_rate.sleep();
  /*
  if(count>1000){
      count =0;
  }
  */
  }

     
  return 0;
}