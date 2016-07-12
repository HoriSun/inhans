#include "appMsg.h"

void APP_Callback(const path_station::APP_ID::ConstPtr& msg)
{//回调函数得到所有的工位：动态添加的工位也会实时得到

  ROS_INFO("the APP value is: [%d]", msg->APP_NODE);
  //保存数据
  int i = msg->APP_NODE;
  app.call[i] = msg->call;
  app.go[i] = msg->go;
  app.APP_NODE[i] = msg->APP_NODE;
  app.station_go[i] = msg->station_go;
  //app.call_time[0].data = msg->call_time;
  //app.stay_time[0] = msg->time;
    
  if(i == station_count - 1) {
    ROS_INFO("the APP info is saved OK!!!");
  }

}  
