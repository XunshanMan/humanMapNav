#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    tf::Quaternion quaternion;
    quaternion.setRPY(0, 0, 0);    //if your URG is not reversed, use this program
    //quaternion.setRPY(M_PI, 0, 0); //if your URG is reversed, use this program 

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(quaternion, tf::Vector3(0.15, 0.0, 0.27)),
        ros::Time::now(),"base_link", "laser_frame"));

    r.sleep();
  }
}
