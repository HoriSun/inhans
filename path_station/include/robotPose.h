#ifndef __PATH_STATION_ROBOT_POSE_H__
#define __PATH_STATION_ROBOT_POSE_H__

#include "public.h"
/**
 * Robot pose 
 */
//robot的实时位置
typedef struct {
  float x;
  float y;
  float theta;
}  Robot_pose;

extern Robot_pose robot_pose;

void robot_pose_Callback(const geometry_msgs::Pose2D::ConstPtr& msg);


#endif
