#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/String.h>
#include <std_msgs/Time.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/PoseArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <cmath>
#include <pthread.h>

#include "path_station/APP_ID.h"
#include "path_station/WorkerStation.h"

using namespace std;

#define N_STATIONS_MAX 100
#define N_APPS_MAX 100

typedef struct __station_info {
  float station_pose[N_STATIONS_MAX][3];
  int   station_time[N_STATIONS_MAX];
  bool  running_status;
  int   station_special[N_STATIONS_MAX];
}  Station_info;

Station_info station_info;


//call　 go time 的保存
typedef struct APP_Information{
  bool   call[N_APPS_MAX];
  bool   go[N_APPS_MAX];
  int    station_go [N_APPS_MAX];
  int    APP_NODE[N_APPS_MAX];
  int    stay_time[N_APPS_MAX];
  ros::Time   call_time[N_APPS_MAX];
  int    priority[N_APPS_MAX]; //优先级，顺序执行，经过计算，重新排布先后执行的工位号
  int    distance[N_APPS_MAX];
} APP;

APP app;

//robot的实时位置
typedef struct __robot_pose {
  float x;
  float y;
  float theta;
}  Robot_pose;

Robot_pose robot_pose;

//分别决定action是否成功和action的类型
bool action_call_flag = false;
bool action_go_flag = true;
bool action_flag = false;

bool station_complete_flag = false;
bool particular_station_complete_flag = false;
int station_count = 0;
int particular_station_count = 0;
int current_call_station = 0; //当前call的工位
std_msgs::Int32 call_count;
geometry_msgs::Pose2D current_robot_pose;

#define DEBUG

void station_info_Callback(const path_station::WorkerStation::ConstPtr& msg)
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
    station_info.station_special[i] = msg->particular_station_indexs.data[i];
    if(i == particular_station_count-1) {
      particular_station_complete_flag = true;
    }
  }
  
  //确保已经完成保存
  if(station_complete_flag == true && particular_station_complete_flag == true) {
    ROS_INFO("the station pose is saved OK!!!");
  }
 
}

void APP_Callback(const path_station::APP_ID::ConstPtr& msg)
{//回调函数得到所有的工位：动态添加的工位也会实时得到

  ROS_INFO("the APP value is: [%d]", msg->APP_NODE);
  //保存数据
  for(int i = 0; i < station_count; i++) {
        if(i == msg->APP_NODE) {
            app.call[i] = msg->call;
            app.go[i] = msg->go;
            app.APP_NODE[i] = msg->APP_NODE;
            app.station_go[i] = msg->station_go;
            //app.call_time[0].data = msg->call_time;
            //app.stay_time[0] = msg->time;
        }
    
        if(i == station_count - 1) {
            ROS_INFO("the APP info is saved OK!!!");
        }
  }

}  
  
//获得小车的当前的位姿
void robot_pose_Callback(const geometry_msgs::Pose2D::ConstPtr& msg) {  //ROS_INFO("the current pose is: [%f %f %f]", msg->x, msg->y, msg->theta);
  robot_pose.x = msg->x;
  robot_pose.y = msg->y;
  robot_pose.theta = msg->theta;
  
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
      if(i == 0){
        ros::Subscriber sub_APP = node.subscribe("APP_ID_1", 10, APP_Callback);

      } else if(i == 1) {
        ros::Subscriber sub_APP = node.subscribe("APP_ID_2", 10, APP_Callback);
      } else if(i == 2) {
        ros::Subscriber sub_APP = node.subscribe("APP_ID_3", 10, APP_Callback);
      } else if(i == 3) {
        ros::Subscriber sub_APP = node.subscribe("APP_ID_4", 10, APP_Callback);
      } else if(i == 4) {
        ros::Subscriber sub_APP = node.subscribe("APP_ID_5", 10, APP_Callback);
      }

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
