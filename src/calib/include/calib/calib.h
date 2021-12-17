#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/core/utility.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/calib3d.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>

#include <vector>

class CalibProc {
    public:
        CalibProc(ros::NodeHandle& n);
        ~CalibProc(){}


        void calibProcCallback(const sensor_msgs::ImageConstPtr& msg);

    private:
        void getBoardCorners(cv::Mat image);

        std::vector<cv::Point2f> pointBuf;
        std::vector<std::vector<cv::Point2f>> imagePoints;
        std::vector<cv::Point3f> objectPointsBuf;
        std::vector<std::vector<cv::Point3f>> objectPoints;

        int min_frame;
        int count;
        bool skip;
        int skipCount;

        ros::Subscriber sub;
};