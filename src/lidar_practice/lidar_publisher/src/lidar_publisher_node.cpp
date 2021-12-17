#include <ros/ros.h>
#include <lidar_msgs/LidarInput.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_publisher");

  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<lidar_msgs::LidarInput>("/lidarInput", 1000);

  ros::Rate loop_rate(20);

  static unsigned long seqCounter = 1U;

  const float lo = 0;
  const float hi = 1;

  while(ros::ok())
  {
    lidar_msgs::LidarInput msg;

    msg.header.stamp = ros::Time::now();
    msg.header.seq = seqCounter;
    msg.header.frame_id = "lidar";

    msg.initialReflectivity = 100;

    double rangeNum = lo + static_cast<double>(rand())/(static_cast<double>(RAND_MAX/(hi-lo)));

    msg.roundTime[0] = rangeNum / 1000000;

    msg.returnReflection[0] = lo + static_cast<double>(rand())/(static_cast<double>(RAND_MAX/(msg.initialReflectivity-lo)));

    pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }
}