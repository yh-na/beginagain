#include "ros/ros.h"
#include "CRobotDrive.h"



int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_drive");

  CRobotDrive rb;
  
  ros::NodeHandle nh;
  ros::Subscriber scan_sub = nh.subscribe("/scan", 10, &CRobotDrive::processLaserScan, &rb);

  ros::NodeHandle nh2;
  ros::Subscriber path_sub = nh2.subscribe("/path_msg",1, &CRobotDrive::processPath, &rb);

  ros::NodeHandle nh3;
  ros::Subscriber odom_sub = nh3.subscribe("/odom", 1, &CRobotDrive::processOdom, &rb);  

  //ros::NodeHandle nh4;
  //ros::Subscriber map_sub = nh4.subscribe("map", 1, &CRobotDrive::mapCallback, &rb);

  ros::spin();

  return 0;
}