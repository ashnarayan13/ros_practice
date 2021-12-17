#include <ros/ros.h>
#include <lidar_processor/lidar_processor.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_processor");

  ros::NodeHandle n;
  LidarProc lidarObj(n);

  ros::spin();

  return 0;
}