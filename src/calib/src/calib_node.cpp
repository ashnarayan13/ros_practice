#include <ros/ros.h>
#include <calib/calib.h>

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "calib_node");

    ros::NodeHandle n;
    CalibProc calib(n);
    ros::spin();

    return 0;
}