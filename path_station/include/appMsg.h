#ifndef __PATH_STATION_APP_MSG_H__
#define __PATH_STATION_APP_MSG_H__

#include "public.h"
/**
 * APP messages
 */

#define N_APPS_MAX 100

//call　 go time 的保存
typedef struct {
  bool   call[N_APPS_MAX];
  bool   go[N_APPS_MAX];
  int    station_go [N_APPS_MAX];
  int    APP_NODE[N_APPS_MAX];
  int    stay_time[N_APPS_MAX];
  ros::Time   call_time[N_APPS_MAX];
  int    priority[N_APPS_MAX]; //优先级，顺序执行，经过计算，重新排布先后执行的工位号
  int    distance[N_APPS_MAX];
} APP;

extern APP app;

void APP_Callback(const path_station::APP_ID::ConstPtr& msg);

/**
 * EXTERNAL
 */
extern int station_count;

#endif
