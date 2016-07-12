#include "callProcess.h"

//对工位优先级进行排序
void station_sort(int distance[], int n) {
    for(int i = 0; i < n-1; i++) {
            int j = i;
            int min = distance[j];
            for(int k=i; k < n; k++) {
                if (distance[k] < min) {
                    j = k;
                    min = distance[k];
                }
            }
            //循环结束后，确定这一轮最小的值
            app.priority[j] = j+1;
        
      /*int t = a[i];
        a[i] = a[j];
        a[j] = t;*/
    }
}




//call队列的处理
int call_record(void) {
    //2,处理call的顺序
    //ros::Time latest_time = ros::Time::now();
    //2.1　得到小车的当前位置坐标
    current_robot_pose.x = robot_pose.x;
    current_robot_pose.y = robot_pose.y;
    current_robot_pose.theta = robot_pose.theta;
    //2.2 计算小车到各个call的工位之间的距离，重新规划路径，每次都重新规划
    //ros::Time latest_time = ros::Time::now();
    for(int i = 0; i < station_count; i++) {
        if(app.call[i] == true && call_count.data < 5) {//只要call的工位才会被处理
          int x = (int)abs(station_info.station_pose[i][0] - current_robot_pose.x);
          int y = (int)abs(station_info.station_pose[i][1] - current_robot_pose.y);
          int distance = (int)sqrt(x*x + y*y);
          app.distance[i] = distance;
          call_count.data++;
          app.call_time[i] = ros::Time::now();
          
        }    
    }
    
    return 0;
} 






//负责计算当前时刻小车到各个工位的距离，以及开始执行call操作
int call_deal(void) {
  //2.3　 根据计算得到的各个call的工位距离小车的距离，确定优先级
  if(call_count.data > 0) {
    station_sort(app.distance, station_count);
    //2.4　处理call,每次只处理最近的一个目标
      for(int i = 0; i < station_count; i++) {
          if(app.distance[i] > 0 && app.priority[i] > 0 ) {
              move_base_msgs::MoveBaseGoal goal;
              goal.target_pose.pose.position.x = station_info.station_pose[app.priority[i]][0];
              goal.target_pose.pose.position.y = station_info.station_pose[app.priority[i]][1];
              //goal.target_pose.pose.position.z = station_info.station_pose[app.priority[i]][2];
              ROS_INFO("call:Sending goal");
              MoveToStartPoint(goal);
              //app.call[i] = false;
              current_call_station = app.priority[i];
              //action_call_flag = true;
              //app.distance[i] = 0;
              //app.priority[i] = 0;
              call_count.data--;
              break;
          }
      }
   } 
  //2.5 清零app.distance app.priority
  for(int i = 0; i < station_count; i++) {
      app.distance[i] = 0;
      app.priority[i] = 0;
  }
  
  return 0;
}



//用于驱动小车到目标地点
int MoveToStartPoint(move_base_msgs::MoveBaseGoal goal) { 
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
      
    MoveBaseClient action("move_base", true);
    
    while(!action.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    
    action.sendGoal(goal);
    action.waitForResult();//等到结果
    //action.cancelAllGoals();

    if(action.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("action success!");
        //action_flag = true;
    }  else {
        ROS_INFO("action failed");
        //action_flag = false;
        return -1;
    }
    //此变量是用于标识当前action任务，不管成功失败都已经执行过了
    action_flag = true;
      
    return 0;
}

//用于取消所有目标
void Stoprunning(void) { 
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
      
    MoveBaseClient action("move_base", true);
    
    while(!action.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    
    action.cancelAllGoals();
    action.waitForResult();
    
    if(action.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("cancel!");
    else
      ROS_INFO("cancel failed");
  
    return;
}
