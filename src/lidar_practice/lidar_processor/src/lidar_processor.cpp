#include <lidar_processor/lidar_processor.h>

LidarProc::LidarProc(ros::NodeHandle& n)
{
  sub = n.subscribe<lidar_msgs::LidarInput>("/lidarInput", 1000, &LidarProc::lidarProcCallback, this);

  pub = n.advertise<lidar_msgs::LidarOutput>("/lidarOutput", 1000);
}

void LidarProc::lidarProcCallback(const lidar_msgs::LidarInput::ConstPtr& msg)
{
  lidar_msgs::LidarOutput outputMsg;

  outputMsg.header = msg->header;
  outputMsg.header.stamp = ros::Time::now();

  outputMsg.intensity = processIntensity(msg);
  outputMsg.range = processRange(msg);

  pub.publish(outputMsg);
}

double LidarProc::processIntensity(const lidar_msgs::LidarInput::ConstPtr& msg)
{
  const double input = msg->initialReflectivity;
  const double output = msg->returnReflection;

  return (output/input)*100.0F;
}

double LidarProc::processRange(const lidar_msgs::LidarInput::ConstPtr& msg)
{
  const double speedOfLight = 299792458.0F;
  const double range = (speedOfLight*msg->roundTime)/2.0F;

  return range;
}