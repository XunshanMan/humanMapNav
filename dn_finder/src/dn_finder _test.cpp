/*
 * succeed in making robots move 0.5 meter.
 * Pay attention: You shouldn't open the rviz window.
 */

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <doornumber/boardArray.h>
#include <doornumber/boardArray.h>
#include <doornumber/board.h>
#include "std_msgs/Bool.h"
#include <iostream>

using namespace std;

// temp parameters
int searchRadius = 3;



typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int move(float x, float y, float w){
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

    goal.target_pose.pose.position.x = 0.5;
    goal.target_pose.pose.orientation.w = 1.0;

//    goal.target_pose.pose.position.x = x;
//    goal.target_pose.pose.position.y = y;
//    goal.target_pose.pose.orientation.w = w;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Hooray, the base moved successfully");
      return 1;
    }
    else
    {
      ROS_INFO("The base failed to move forward for some reason");
      return 0;
    }

}

void subBoardArray(const doornumber::boardArray::ConstPtr& msg)
{ for (int i=0; i<msg->boardArray.size(); ++i)
    {
        const doornumber::board &board = msg->boardArray[i];
        ROS_INFO_STREAM("text: " << board.text << "confidence: " << board.confidence);
    }
}

void testMove(){
    float x,y,w;
    ros::Rate loop_rate(1);

    while(1){
        printf("please input command: mv(x,y,w)\n");
        scanf("mv(%f,%f,%f)", &x, &y, &w);
        printf("get command: mv(%f, %f, %f)\n", x,y,w);
        if(-5<x && x<5 && -5<y && y<5 && -2<w &&w <2)
        {
            printf("running command.\n");
            move(x,y,w);
        }
        else
            printf("command error.\n");
       loop_rate.sleep();
    }
}

int main(int argc, char** argv){
//  ROS_INFO << "doornumber_finder program: using doornumber information "
//        <<endl<<  "to control the robots find the door number and move toward it."
//         << endl;

  ros::init(argc, argv, "dn_finder");

  ros::NodeHandle n;
  /* get a door infor*/
  ros::Subscriber sub = n.subscribe("boardArray", 1000, subBoardArray);

  ros::Publisher command_pub = n.advertise<std_msgs::Bool>("dn_finder_command", 10);

  ros::Rate loop_rate(1);
  std_msgs::Bool modeState;
  modeState.data = true;

  //testMove();
  move(1,1,1);
  goto END;

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */

    ROS_INFO("state: %d", modeState.data);

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    command_pub.publish(modeState);

    ros::spinOnce();

    loop_rate.sleep();

    modeState.data=!modeState.data;
  }

  ros::spin();

END:  return 0;
}

