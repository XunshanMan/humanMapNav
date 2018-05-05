#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <doornumber/boardArray.h>
#include <doornumber/board.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"

#include <iostream>

#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <string>
#include <sstream>

#include <math.h>

#define INITIAL_ORIENTATION_Z 0.999
#define INITIAL_ORIENTATION_W 0.046

#define PI 3.14159

enum robotState{
    STATE_NONE = 0,
    STATE_RUNNING = 1,
    STATE_FINISHED = 2
};
robotState _state = STATE_NONE;

using namespace std;

// temp parameters
int searchRadius = 3;

ros::Publisher command_pub;
ros::Publisher result_pub;
ros::Publisher home_pub;

string doorNumberString;

bool waitForDetect = 0;

// For Sending command to move_base
float command_x = 0;
float command_y = 0;
float command_angle = 0;
geometry_msgs::Quaternion odom_quat;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int send_move_command(MoveBaseClient ac, float x, float y, float oz, float ow){

    move_base_msgs::MoveBaseGoal goal;
    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

//    goal.target_pose.pose.position.x = 0.5;
//    goal.target_pose.pose.orientation.w = 1.0;

    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;

    goal.target_pose.pose.orientation.z = oz;
    goal.target_pose.pose.orientation.w = ow;

    ros::Rate loop_rate(1);
    ROS_INFO("Sending goal * 3");
    ac.sendGoal(goal);
    loop_rate.sleep();
    ac.sendGoal(goal);
    loop_rate.sleep();

    ac.sendGoal(goal);

}

int move(float x, float y, float oz, float ow){
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);
    //wait for the action server to come up
    if(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("[But still send]]Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

//    goal.target_pose.pose.position.x = 0.5;
//    goal.target_pose.pose.orientation.w = 1.0;

    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;

    goal.target_pose.pose.orientation.z = oz;
    goal.target_pose.pose.orientation.w = ow;

    ros::Rate loop_rate(1);
    ROS_INFO("Sending goal * 3");
    ac.sendGoal(goal);
    loop_rate.sleep();
    ac.sendGoal(goal);
    loop_rate.sleep();

    ac.sendGoal(goal);

    ROS_INFO("wait For Result...");
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Hooray, the base moved successfully");
      printf("Current State: %s\n", ac.getState().toString().c_str());
      return 1;
    }
    else
    {
      ROS_INFO("The base failed to move forward for some reason");
      printf("Current State: %s\n", ac.getState().toString().c_str());
      return 0;
    }

}

int move_robot(float x, float y, float oz, float ow){
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);
    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();

//    goal.target_pose.pose.position.x = 0.5;
//    goal.target_pose.pose.orientation.w = 1.0;

    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;

    goal.target_pose.pose.orientation.z = oz;
    goal.target_pose.pose.orientation.w = ow;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ROS_INFO("wait For Result...");
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Hooray, the base moved successfully");
      printf("Current State: %s\n", ac.getState().toString().c_str());
      return 1;
    }
    else
    {
      ROS_INFO("The base failed to move forward for some reason");
      printf("Current State: %s\n", ac.getState().toString().c_str());
      return 0;
    }

}

int openDetect(){
    std_msgs::Bool modeState;
    modeState.data = true;

    ROS_INFO("now send the state: %d", modeState.data);

    command_pub.publish(modeState);

    return 1;
}

int stopDetect(){
    std_msgs::Bool modeState;
    modeState.data = false;

    ROS_INFO("now send the state: %d", modeState.data);

    command_pub.publish(modeState);

    return 1;
}

geometry_msgs::Quaternion createQuaternionMsgFromYaw(float w){
    geometry_msgs::Quaternion q;
    q.x = 0;
    q.y = 0;
    q.z = sin(w/2.0);
    q.w = cos(w/2.0);
    return q;
}

//// initialize the home point
//void setHome(float x, float y, float w)
//{
//    //Frame:map, Position(8.362, 4.945, 0.000),
//    //Orientation(0.000, 0.000, 0.999, 0.046) = Angle: 3.050

//    //For the New HALF Map
//    //Frame:map, Position(9.018, -3.241, 0.000),
//    //Orientation(0.000, 0.000, 0.873, 0.488) = Angle: 2.123

//    //For the 3L_Full Map
//    //Frame:map, Position(7.540, -4.674, 0.000),
//    //Orientation(0.000, 0.000, 0.789, 0.614) = Angle: 1.819

//    // calculate the orientation
//    geometry_msgs::Quaternion odom_quat = createQuaternionMsgFromYaw(w);

//    ROS_INFO("quaternion: x %f, y %f, z %f, w %f", odom_quat.x, odom_quat.y, odom_quat.z, odom_quat.w);

//    geometry_msgs::PoseWithCovarianceStamped msg_poseinit;
//    msg_poseinit.header.frame_id = "map";
//    msg_poseinit.header.stamp = ros::Time::now();
//    msg_poseinit.pose.pose.position.x = x;
//    msg_poseinit.pose.pose.position.y = y;
//    msg_poseinit.pose.pose.position.z = 0.0;
//    msg_poseinit.pose.pose.orientation.x = 0.0;
//    msg_poseinit.pose.pose.orientation.y = 0.0;
//    msg_poseinit.pose.pose.orientation.z = 0.789;
//    msg_poseinit.pose.pose.orientation.w = 0.614;
//    home_pub.publish(msg_poseinit);
//    ros::Duration(1.0).sleep();
//    home_pub.publish(msg_poseinit);
//    ros::Duration(1.0).sleep();
//    home_pub.publish(msg_poseinit);
//    ros::Duration(1.0).sleep();
//}

void controlToBoard(const doornumber::board board){
    float PARAM_CAMSIGHT_WIDTH = 2 * 30 * tan(PI/6);
    float distance = board.tlx * PARAM_CAMSIGHT_WIDTH;

    ROS_INFO("Moving Robot to the Board. distance:%f", distance);
    move_robot(distance, 0,sin(PI/4),cos(PI/2));   // turn left for 90
}

void subBoardArray(const doornumber::boardArray::ConstPtr& msg)
{
    // when the doornumber node transmits back the information.
    ROS_INFO("dn_finder get board information.");
    doornumber::boardArray resultBoards;
    for (int i=0; i<msg->boardArray.size(); ++i)
    {
        const doornumber::board &board = msg->boardArray[i];
        ROS_INFO_STREAM("text: " << board.text << "confidence: " << board.confidence);
        string::size_type idx;
        idx=board.text.find(doorNumberString);//在a中查找b.

        if(idx != string::npos){
            ROS_INFO("Final Goal is %s", doorNumberString.c_str());
            ROS_INFO("Get the doornumber!!!(%%%.1f)", board.confidence);
            resultBoards.boardArray.push_back(board);
        }

    }
    waitForDetect = 0;

    result_pub.publish(resultBoards);
    if(resultBoards.boardArray.size() > 0){

        // control robot to the board.
        controlToBoard(resultBoards.boardArray[0]);
        // end the searching command.
    }

    //stopDetect();
}

void testMove(){
    float x,y,z,w;
    ros::Rate loop_rate(1);

    while(1){
        printf("please input command: mv(x,y,w,z)\n");
        scanf("mv(%f,%f,%f,%f)", &x, &y, &w,&z);
        printf("get command: mv(%f, %f, %f, %f)\n", x,y,w,z);
        if(-5<x && x<5 && -5<y && y<5 && -2<w &&w <2)
        {
            printf("running command.\n");
            move(x,y,z,w);
        }
        else
            printf("command error.\n");
       loop_rate.sleep();
    }
}

// initialize the home point
void setHomeTest( ros::Publisher pub)
{
    //Frame:map, Position(8.362, 4.945, 0.000),
    //Orientation(0.000, 0.000, 0.999, 0.046) = Angle: 3.050

    //For the New HALF Map
    //Frame:map, Position(9.018, -3.241, 0.000),
    //Orientation(0.000, 0.000, 0.873, 0.488) = Angle: 2.123

    //For the 3L_Full Map
    //Frame:map, Position(7.540, -4.674, 0.000),
    //Orientation(0.000, 0.000, 0.789, 0.614) = Angle: 1.819

    geometry_msgs::PoseWithCovarianceStamped msg_poseinit;
    msg_poseinit.header.frame_id = "map";
    msg_poseinit.header.stamp = ros::Time::now();
    msg_poseinit.pose.pose.position.x = 7.540;
    msg_poseinit.pose.pose.position.y = -4.674;
    msg_poseinit.pose.pose.position.z = 0.0;
    msg_poseinit.pose.pose.orientation.x = 0.0;
    msg_poseinit.pose.pose.orientation.y = 0.0;
    msg_poseinit.pose.pose.orientation.z = 0.789;
    msg_poseinit.pose.pose.orientation.w = 0.614;
    pub.publish(msg_poseinit);
    ros::Duration(1.0).sleep();
    pub.publish(msg_poseinit);
    ros::Duration(1.0).sleep();
    pub.publish(msg_poseinit);
    ros::Duration(1.0).sleep();
}

void test(ros::NodeHandle n){

    ros::Rate loop_rate(0.2);
    ros::Rate loop_rate_1(1);
    ros::Rate loop_rate_3(0.33);

    if(0)
      ROS_INFO("Using the default home point...");
    else
    // initial home point
    {
        ROS_INFO("Set the home point...");
        ros::Publisher pub_initialpose = n.advertise<geometry_msgs::
                PoseWithCovarianceStamped>("/initialpose", 10);
        setHomeTest(pub_initialpose);
    }
    ROS_INFO("Now begin searching.");


    for(int i=0; i<5; i++){
  //      move(1, 0, INITIAL_ORIENTATION_Z, INITIAL_ORIENTATION_W);
        move(0.5, 0, 0, 1);
        loop_rate.sleep();        // make sure it stop
        loop_rate.sleep();        // make sure it stop
        openDetect();
        waitForDetect = 1;
        while(waitForDetect){
            ROS_INFO("Wait to receive detect result.");
            ros::spinOnce();

            loop_rate_1.sleep();
        }
        loop_rate.sleep();

    }
}

//void moveOnMap(float x, float y){
//    printf("Now robot is moving to (%f,%f) on the Map. \n", x, y);

//    //tell the action client that we want to spin a thread by default

//    move_base_msgs::MoveBaseGoal goal;

//    //we'll send a goal to the robot to move 1 meter forward
//    goal.target_pose.header.frame_id = "base_link";
//    goal.target_pose.header.stamp = ros::Time::now();

//    goal.target_pose.pose.position.x = 1.0;
//    goal.target_pose.pose.orientation.w = 1.0;

//    ROS_INFO("Sending goal");
//    ac.sendGoal(goal);

//    ac.waitForResult();

//    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//      ROS_INFO("Hooray, the base moved 1 meter forward");
//    else
//      ROS_INFO("The base failed to move forward 1 meter for some reason");

//    return;
//}

void subMapCommand(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    ros::Rate loop_rate_1(1);

    if(_state == STATE_NONE){
        _state = STATE_RUNNING;

        // set home...
        float homex = msg->data[2];
        float homey = msg->data[3];
        float homeangle = msg->data[4];
        int intnumber = int(msg->data[5]);
        stringstream stream;     //声明一个stringstream变量
        stream << intnumber;     //向stream中插入整型数1234
        stream >> doorNumberString;     //从steam中提取刚插入的整型数   并将其赋予变量str完成整型数到string的转换

        //This function is useless logically. Just try to comment it.
        //setHome(homex, homey, homeangle);
        loop_rate_1.sleep();
    }
    float x = msg->data[0];
    float y = msg->data[1];

    command_x = x;
    command_y = y;
    command_angle = msg->data[4];
    printf("get command from UImap, x:%f y:%f angle:%f\n", x, y, command_angle);

    odom_quat = createQuaternionMsgFromYaw(command_angle);
    ROS_INFO("quaternion: x %f, y %f, z %f, w %f", odom_quat.x, odom_quat.y, odom_quat.z, odom_quat.w);

}

int main(int argc, char** argv){
//  ROS_INFO << "doornumber_finder program: using doornumber information "
//        <<endl<<  "to control the robots find the door number and move toward it."
//         << endl;

// sethome
  ros::init(argc, argv, "dn_finder");

  ros::NodeHandle n;
  /* get a door infor*/
  ros::Subscriber sub = n.subscribe("boardArray", 1000, subBoardArray);

  command_pub = n.advertise<std_msgs::Bool>("dn_finder_command", 10);

  // for connecting with UImap_node
  result_pub = n.advertise<doornumber::boardArray>("searchResult", 1);
  ros::Subscriber mapCommand_sub = n.subscribe("searchCommand", 1, subMapCommand);

  home_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);
//  if(1)
//    ROS_INFO("Home will be set when first command comes....");
//  else
//  // initial home point
//  {
//      ROS_INFO("Set the home point...");
//      ros::Publisher pub_initialpose = n.advertise<geometry_msgs::
//              PoseWithCovarianceStamped>("/initialpose", 10);
//      setHomeTest(pub_initialpose);
//  }
  ROS_INFO("Now wait for searching command.");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  //wait for the action server to come up
  if(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("[But still send]]Waiting for the move_base action server to come up");
  }

  float command_now_x = 0;
  float command_now_y = 0;
  ros::Rate loop_rate_1(1);

  while(ros::ok()){
      // send and wait for move_base commend here.
      if(command_now_x==command_x && command_now_y==command_y)
      {
          ros::spinOnce();
          continue;
      }
beginAgain: printf("send new command.\n");
//      send_move_command(ac, command_x, command_y, 1, 0);
      move_base_msgs::MoveBaseGoal goal;

      if( abs(command_x) < 0.01 && abs(command_y) < 0.01)
      {
          printf("SYSTEM HAS BEEN CANCELED.\n");
          ac.cancelGoal();
          command_now_x = command_x;
          command_now_y = command_y;
          ros::spinOnce();

          continue;
      }

      //we'll send a goal to the robot to move 1 meter forward
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose.position.x = command_x;
      goal.target_pose.pose.position.y = command_y;

      goal.target_pose.pose.orientation.x = 0;
      goal.target_pose.pose.orientation.y = 0;

      goal.target_pose.pose.orientation.z = odom_quat.z;
      goal.target_pose.pose.orientation.w = odom_quat.w;

      ros::Rate loop_rate(1);
      ROS_INFO("Sending goal * 3");
      ac.sendGoal(goal);
      loop_rate.sleep();
      ac.sendGoal(goal);
      loop_rate.sleep();

      ac.sendGoal(goal);


      ROS_INFO("wait For Result...");
      command_now_x = command_x;
      command_now_y = command_y;

      while(!ac.waitForResult(ros::Duration(2))){ //Check for every 2 seconds
          ros::spinOnce();
          if(command_now_x!=command_x || command_now_y!=command_y)
          {
              ac.cancelGoal();
              goto beginAgain;
          }
      }


      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Hooray, the base moved successfully");
        printf("Current State: %s\n", ac.getState().toString().c_str());
//        return 1;

        loop_rate_1.sleep();
        loop_rate_1.sleep();
        loop_rate_1.sleep();
        loop_rate_1.sleep();
        // wait for 4 seconds.

        openDetect();
        waitForDetect = 1;
        while(waitForDetect){
            ROS_INFO("Wait to receive detect result.");
            ros::spinOnce();

            loop_rate_1.sleep();
        }

        ROS_INFO("get result now.");

      }
      else
      {
        ROS_INFO("The base failed to move forward for some reason");
        printf("Current State: %s\n", ac.getState().toString().c_str());
//        return 0;
      }

      ros::spinOnce();
      //test(n);
  }

  return 0;
}

