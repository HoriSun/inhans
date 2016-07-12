#ifndef __PATH_STATION_STATION_SERVER_H__
#define __PATH_STATION_STATION_SERVER_H__

#include "appMsg.h"
#include "robotPose.h"
#include "stationInfo.h"
#include "callProcess.h"

/**
 * APP message
 */
APP app;

/**
 * Robot Pose
 */
Robot_pose robot_pose;

/**
 * Station Information
 */
Station_info station_info;


// available CALLs remain
std_msgs::Int32 call_count;

// how many stations
int station_count = 0;
int particular_station_count = 0;

// flags
bool station_complete_flag = false;
bool particular_station_complete_flag = false;



/**
 * Call process
 */
int current_call_station = 0; //当前call的工位
geometry_msgs::Pose2D current_robot_pose;

//分别决定action是否成功和action的类型
bool action_call_flag = false;
bool action_go_flag = true;
bool action_flag = false;


#endif
