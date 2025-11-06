/*
 * @Author: Yuhao Zhou
 * @Date: 2023-02-26 18:05:20
 * @LastEditors: Yuhao Zhou
 * @LastEditTime: 2023-05-22 17:24:18
 * @Description: Robot-patient
 * @FilePath: /real_ws/src/UR_Real/Controllers_Algorithms/universal_robot_control/cartesian_velocity_controller/src/sub.cpp
 */
#include "ros/ros.h"
#include "cartesian_state_msgs/PoseTwist.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

static ros::Subscriber sub;
static ros::Publisher pub;
void chatterCallback(const cartesian_state_msgs::PoseTwist::ConstPtr& msg){

  // Unity has different coordinate system compared to ROS:
  // https://github.com/siemens/ros-sharp/wiki/Dev_ROSUnityCoordinateSystemConversion

  geometry_msgs::Quaternion quat_msg;
  quat_msg.x = msg->pose.orientation.x;
  quat_msg.y = msg->pose.orientation.y;
  quat_msg.z = msg->pose.orientation.z;
  quat_msg.w = msg->pose.orientation.w;
  double r,p,y;
  tf::Quaternion quat(quat_msg.x,quat_msg.y,quat_msg.z,quat_msg.w);
  tf::Matrix3x3(quat).getRPY(r, p, y);
  tf2::Quaternion q_orig, q_rot, q_new;
  q_orig.setRPY(r, p, y);
  q_rot.setRPY(0.0, 0.0, 0.0);
  q_new = q_rot * q_orig;
  q_new.normalize();

  geometry_msgs::PoseStamped unitymsg;
  unitymsg.header = msg->header;
  unitymsg.pose.position.x = msg->pose.position.x;
  unitymsg.pose.position.y = msg->pose.position.y;
  unitymsg.pose.position.z = msg->pose.position.z + 0.85; // add the base height, plz modify based on specific human model
  unitymsg.pose.orientation.x = q_new.x();
  unitymsg.pose.orientation.y = q_new.y();
  unitymsg.pose.orientation.z = q_new.z();
  unitymsg.pose.orientation.w = q_new.w();

  pub.publish(unitymsg);  
}

int main(int argc, char **argv){
  ROS_INFO("ROS Publishing loop .................");
  ros::init(argc, argv, "transfer");
  ros::NodeHandle n;
  sub = n.subscribe("/cartesian_velocity_controller_sim/ee_state", 5, chatterCallback);
  pub = n.advertise<geometry_msgs::PoseStamped>("/unity", 5, chatterCallback);
  ros::spin();
  return 0;

}
