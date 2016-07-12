#ifndef __PATH_STATION_CALL_PROCESS_H__
#define __PATH_STATION_CALL_PROCESS_H__

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "public.h"
#include "appMsg.h"
#include "stationInfo.h"
#include "robotPose.h"

/**
 * Call process
 */
extern int current_call_station;// = 0; //当前call的工位
extern geometry_msgs::Pose2D current_robot_pose;
//分别决定action是否成功和action的类型
extern bool action_call_flag;
extern bool action_go_flag;
extern bool action_flag;

void station_sort(int distance[], int n);
int call_record(void);
int call_deal(void);
int MoveToStartPoint(move_base_msgs::MoveBaseGoal goal);
void Stoprunning(void);

/**
 * EXTERNAL
 */
extern APP app;
extern geometry_msgs::Pose2D current_robot_pose;
extern std_msgs::Int32 call_count;
extern Station_info station_info;
extern int station_count;

#endif
