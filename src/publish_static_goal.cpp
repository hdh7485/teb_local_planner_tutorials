#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "publish_static_goal");

  //tell the action client that we want to spin a thread by default
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("/rviz_goal", 100);

  geometry_msgs::PoseStamped goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.header.frame_id = "odom";
  goal.header.stamp = ros::Time::now();

  goal.pose.position.x = 0.0;
  goal.pose.position.y = 100.0;
  goal.pose.orientation.w = 1.0;

  ros::Rate loop_rate(1);
  while(ros::ok()){
    pub.publish(goal);
    loop_rate.sleep();
  }

  return 0;
}
