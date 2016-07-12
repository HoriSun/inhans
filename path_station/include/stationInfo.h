#ifndef __PATH_STATION_STATION_INFO_H__
#define __PATH_STATION_STATION_INFO_H__

#include "public.h"
/**
 * Station information
 */
#include <std_msgs/Int32.h>
#define N_STATIONS_MAX 100

typedef struct __station_info {
  float station_pose[N_STATIONS_MAX][3]; // position
  int   station_time[N_STATIONS_MAX]; // stay time
  bool  running_status;  // ???
  int   station_particular[N_STATIONS_MAX]; // particular stations
}  Station_info;

extern Station_info station_info;

// available CALLs remain
extern std_msgs::Int32 call_count;

// how many stations
extern int station_count;
extern int particular_station_count;

// flags
extern bool station_complete_flag;
extern bool particular_station_complete_flag;

void station_info_Callback(const path_station::WorkerStation::ConstPtr& msg);

#endif
