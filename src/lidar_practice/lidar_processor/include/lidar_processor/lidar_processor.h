#include <ros/ros.h>
#include <lidar_msgs/LidarInput.h>
#include <lidar_msgs/LidarOutput.h>

class LidarProc
{
  public:
    LidarProc(ros::NodeHandle& n);
    ~LidarProc() {}

    void lidarProcCallback(const lidar_msgs::LidarInput::ConstPtr& msg);

  private:
    double processRange(const lidar_msgs::LidarInput::ConstPtr& msg);

    double processIntensity(const lidar_msgs::LidarInput::ConstPtr& msg);

    ros::Subscriber sub;
    ros::Publisher pub;
};