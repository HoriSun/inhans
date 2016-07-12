#include "robotPose.h"
//获得小车的当前的位姿
void robot_pose_Callback(const geometry_msgs::Pose2D::ConstPtr& msg) {  
  //ROS_INFO("the current pose is: [%f %f %f]", msg->x, msg->y, msg->theta);
  robot_pose.x = msg->x;
  robot_pose.y = msg->y;
  robot_pose.theta = msg->theta;  
}
