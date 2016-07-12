
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
