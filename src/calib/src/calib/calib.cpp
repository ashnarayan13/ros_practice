#include <calib/calib.h>
#include <sensor_msgs/image_encodings.h>

CalibProc::CalibProc(ros::NodeHandle& n)
{
    sub = n.subscribe<sensor_msgs::Image>("/camera/image_color", 1, &CalibProc::calibProcCallback, this);

    min_frame = 60;

    count = 0;

    skipCount = 0;
    skip = false;

    imagePoints.clear();
    objectPoints.clear();
    pointBuf.clear();
    objectPointsBuf.clear();

    int width = 8;
    int height = 6;
    float sq_size = 2.4f;

    for(int i=0; i<height; i++)
    {
        for(int j=0; j<width; j++)
        {
            cv::Point3f tmpPt;
            tmpPt.x = j*sq_size;
            tmpPt.y = i*sq_size;
            tmpPt.z = 0.0f;

            objectPointsBuf.push_back(tmpPt);
        }
    }
}


void CalibProc::calibProcCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // Read msg
    // Convert to OpenCV
    // Find chessboard corners
    // process image

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "mono8");

    int chessBoardFlags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;

    bool found = false;

    found = cv::findChessboardCorners(cv_ptr->image, cv::Size(8,6), pointBuf, chessBoardFlags);

    if(found)
    {
        cv::cornerSubPix(cv_ptr->image, pointBuf, cv::Size(5,5), cv::Size(-1,-1), cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 30, 0.0001));

        if(skip)
        {
            skipCount++;

            if(skipCount > min_frame)
            {
                skipCount = 0;
                skip = false;
            }

            ROS_INFO("SKIPPED \n");
        }
        else
        {
            ROS_INFO("Processed %d", count);

            skip = true;
            skipCount = 0;
            imagePoints.push_back(pointBuf);
            objectPoints.push_back(objectPointsBuf);
            count++;
        }
    }

    if(sub.getNumPublishers() == 0)
    {
        ROS_INFO("No one is publishing! \n");
        ROS_INFO("Calibration started!\n");

        cv::Mat cameraMat = cv::Mat::eye(3,3,CV_64F);
        cv::Mat distCoeff = cv::Mat::zeros(8, 1, CV_64F);
        std::vector<cv::Mat>tvecs, rvecs;
        double rms = cv::calibrateCamera(objectPoints, imagePoints, cv::Size(1920, 1080), cameraMat, distCoeff, rvecs, tvecs, cv::CALIB_USE_LU);

        ROS_INFO("Calibration complete. Writing to file \n");

        cv::FileStorage fs("/home/nomad/projects/skill-lync/catkin_ws/src/calib/output.yml", cv::FileStorage::WRITE);
        fs << "image_width" << 1920;
        fs << "image_height" << 1080;
        fs << "camera_matrix" << cameraMat;
        fs << "distortion_coefficients" << distCoeff;

        sub.shutdown();
        ros::shutdown();
    }
}