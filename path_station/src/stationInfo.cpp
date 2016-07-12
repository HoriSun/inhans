#include "stationInfo.h"
void station_info_Callback(const path_station::WorkerStation::ConstPtr& msg)
/**
 * geometry_msgs/PoseArray  positions
 * std_msgs/Int32MultiArray ids
 * std_msgs/Int32MultiArray stay_times
 * bool                     running_status
 * std_msgs/Int32MultiArray particular_station_indexs
 **/
{//回调函数得到所有的工位：动态添加的工位也会实时得到
  
  //得到数组的长度
  station_count = msg->stay_times.data.size();
  call_count.data = station_count;
  if(station_count > 0) {
#ifdef DEBUG
    ROS_INFO("the station pose is: [%f], size is %d\n", 
             msg->positions.poses[0].position.x, 
             msg->stay_times.data.size()
             /*sizeof(msg->positions.poses)/sizeof(msg->positions.poses[0])*/
             );
    ROS_INFO("the station pose is: [x = %f], [y = %f], [theta = %f]\n", 
             msg->positions.poses[0].position.x, 
             msg->positions.poses[0].position.y, 
             msg->positions.poses[0].position.z);
    ROS_INFO("the station pose is: [station one　 = %d], [station two = %d], [station three = %d]\n", 
             msg->ids.data[0], 
             msg->ids.data[1], 
             msg->ids.data[2]);
    ROS_INFO("the station pose is: [station one time = %d], [station two time = %d], [station three time = %d]\n", 
             msg->stay_times.data[0], 
             msg->stay_times.data[1], 
             msg->stay_times.data[2]);
    ROS_INFO("the station pose is: [running_status = %d]\n", 
             msg->running_status);
    ROS_INFO("the station pose is: [particular index1 = %d], [particular index2 = %d], [particular index3 = %d]\n", 
             msg->particular_station_indexs.data[0], 
             msg->particular_station_indexs.data[1], 
             msg->particular_station_indexs.data[2]);
#endif  
  }
  
  //急停状态位
  station_info.running_status = msg->running_status;         
  //保存数据
  for(int i = 0; i < station_count; i++) {
    
    //位姿信息
    station_info.station_pose[i][0] = msg->positions.poses[i].position.x;
    station_info.station_pose[i][1] = msg->positions.poses[i].position.y;
    station_info.station_pose[i][2] = msg->positions.poses[i].position.z;
    //每个工位停留的时间
    station_info.station_time[i] = msg->stay_times.data[i];
    //确认已经复制完毕
    if(i == station_count-1) {
      station_complete_flag = true;
    }
    
  }
  
  //保存数据特殊工位的信息
  particular_station_count = msg->particular_station_indexs.data.size();
  for(int i = 0; i < particular_station_count; i++) {
    //特殊工位下标
    station_info.station_particular[i] = msg->particular_station_indexs.data[i];
    if(i == particular_station_count-1) {
      particular_station_complete_flag = true;
    }
  }
  
  //确保已经完成保存
  if(station_complete_flag == true && particular_station_complete_flag == true) {
    ROS_INFO("the station pose is saved OK!!!");
  }
 
}


