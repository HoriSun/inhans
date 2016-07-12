// 1. Rename `particular_station_count` as `particular_station_count`

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>

#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/PoseArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <pthread.h>

//#define DEBUG

#include "station_server.h"


#define APP_ID_PREFIX "APP_ID_"


//创建一个C++线程
void* func(void* arg)
{
   //用于停止正在进行的action
   if(station_info.running_status == false) {
      //move_base_msgs::MoveBaseGoal goal;
      //goal.target_pose.pose.position.x = current_robot_pose.x;
      //goal.target_pose.pose.position.y = current_robot_pose.y;
      //goal.target_pose.pose.position.z = station_info.station_pose[app.priority[i]][2];
      //ROS_INFO("call:Sending goal");
      //MoveToStartPoint(goal);
      Stoprunning();
   }
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "robot_run_path"); //execute program name
  ros::NodeHandle node; //NodeHandle is the main access point to communications with the ROS system
  //创建线程
  pthread_t id;
  int ret; 
  ret = pthread_create(&id, NULL, *func, NULL);
  if(ret != 0) {
      ROS_INFO("create thread failure!");
      exit(1);
  }
  //int ret= pthread_create(&id,　NULL,　(void*)func, NULL);
  //当下call的数量
  ros::Publisher pub_call =  node.advertise<std_msgs::Int32>("call_count", 10);
   
  //利用回调函数得到所有station的坐标
  ros::Subscriber sub_station_info = node.subscribe("worker_stations", 10, station_info_Callback);
  //订阅手机端app发出的节点，要遍历一遍所有的手机app节点
  //APP_ID_xx节点名字,　call的数量，　go的数量，  ID号　　APP节点的总数,由于遍历时是按照顺序遍历的，因此所有的app节点可以使用同一个回调函数
  

  for(int i = 0; i < station_count; i++)
  {
    //The format is `APP_ID_<number>`
    char APP_ID_NameBuffer[sizeof(APP_ID_PREFIX)+10] = {0};
    sprintf(APP_ID_NameBuffer, APP_ID_PREFIX"%d\0", i);
    ros::Subscriber sub_APP = node.subscribe(APP_ID_NameBuffer, 10, APP_Callback);
  }
   
  //得到小车的位置信息
  ros::Subscriber sub_robot_pose = node.subscribe("robot_pose_in_map", 10, robot_pose_Callback);
   
  //处理完go后，再处理call.call是有队列的，因此这里先计算小车此时距离各个工位的距离，然后再进行下一步操作，遵从就近原则
  //在处理完第一个call之后，马上就go操作，同时把call写为false,回调函数接受手机端的call
  
  ros::Rate loop_rate(10);
  
  //call_count初始化
  call_count.data = 0;
  //清零
  for(int i = 0; i < station_count; i++) {
        app.distance[i] = 0;
        app.priority[i] = 0;
  }
  
  while (ros::ok()){
    //１，得到station的各个工位坐标, 延迟0.5秒处理
    //ros::Duration d(0.5);
    //d.sleep();
    //1, 处理紧急停止的情况
    if(station_info.running_status == true) {
        
        call_record();
    }
      
    //3,发布call_count
    pub_call.publish(call_count);
    //4,　处理call队列
    if(action_flag == false) {
        call_deal();
    }
    //4, 查看是否有APP go
    //确定已经收到了最新的工位信息
    if(station_complete_flag == true && particular_station_complete_flag == true) {
        for(int i = 0; i < station_count; i++) {
            if(app.go[i] == true && app.APP_NODE[i] == current_call_station && action_flag == true) {
                move_base_msgs::MoveBaseGoal goal;
                goal.target_pose.pose.position.x = station_info.station_pose[app.station_go[i]][0];
                goal.target_pose.pose.position.y = station_info.station_pose[app.station_go[i]][1];
                //goal.target_pose.pose.position.z = station_info.station_pose[i][2];
                ROS_INFO("go:Sending goal");
                MoveToStartPoint(goal);
                //go相应的工位置为false,call的位置也置为false
                //if(station_info.running_status == true) {
                app.go[i] = false;
                app.call[i] = false;
                action_flag = false;
                    //action_go_flag = true;
                  
                //}
                
                //app.call[i] = false;
                break;
            } /*else if(app.go[i] == false && (app.call_time[i] + ros::Duration((double)station_info.station_time[i]) > ros::Time::now()) && action_flag == true) {
                move_base_msgs::MoveBaseGoal goal;
                goal.target_pose.pose.position.x = station_info.station_pose[i][0];
                goal.target_pose.pose.position.y = station_info.station_pose[i][1];
                //goal.target_pose.pose.position.z = station_info.station_pose[i][2];
                ROS_INFO("time is used over,go:Sending goal");
                MoveToStartPoint(goal);
                //go相应的工位置为false,call的位置也置为true
                if(station_info.running_status == true) {
                    app.go[i] = false;
                    app.call[i] = false;
                    action_flag = false;
                } 
                
                break;
            }*/
        } 
    }
    
    //5, 小车循环运行在各个工位
    if(call_count.data <= 0) {
        if(action_flag == false) {
            static int i = 0;
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.pose.position.x = station_info.station_pose[i][0];
            goal.target_pose.pose.position.y = station_info.station_pose[i][1];
            //goal.target_pose.pose.position.z = station_info.station_pose[i][2];
            ROS_INFO("call is used over,go:Sending goal");
            MoveToStartPoint(goal);
            if(i < station_count) 
                i++;
          
            action_flag = false;
        }
    
    }  
   
    ros::spinOnce();//为了使得订阅者的回调函数被调用
    loop_rate.sleep();
       
  }
  
  return 0;

}
