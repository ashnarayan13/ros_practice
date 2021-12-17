#include <ros/ros.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <tf2/transform_datatypes.h>

geometry_msgs::TransformStamped currTransform;
bool found = false;
ros::Publisher pub;

void transformCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    ROS_INFO("Entered callback. Flag %d", found);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    tfBuffer.canTransform("imu_rot", "imu_link", ros::Time(), ros::Duration(1.0));

    if(!found)
    {
        try 
        {
            geometry_msgs::TransformStamped transform;
            transform = tfBuffer.lookupTransform("imu_rot", "imu_link", ros::Time());

            ROS_INFO("x:%f, y:%f, z:%f", transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);

            ROS_INFO("q1:%f, q2:%f, q3:%f, q4:%f", transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w);

            currTransform = transform;
            found = true;
        }
        catch (tf2::LookupException &ex)
        {
            ROS_INFO("%s", ex.what());
        }
    }
    else
    {
        sensor_msgs::Imu op;
        op.header = msg->header;
        op.header.frame_id = "imu_rot";
        op.header.stamp = ros::Time::now();

        tf2::Vector3 vel;
        vel.setX(msg->angular_velocity.x);
        vel.setY(msg->angular_velocity.y);
        vel.setZ(msg->angular_velocity.z);


        tf2::Vector3 mAcc;
        mAcc.setX(msg->linear_acceleration.x);
        mAcc.setY(msg->linear_acceleration.y);
        mAcc.setZ(msg->linear_acceleration.z);

        tf2::Quaternion qt;
        qt.setX(currTransform.transform.rotation.x);
        qt.setY(currTransform.transform.rotation.y);
        qt.setZ(currTransform.transform.rotation.z);
        qt.setW(currTransform.transform.rotation.w);

        tf2::Vector3 res = tf2::quatRotate(qt, vel);

        tf2::Vector3 accel = tf2::quatRotate(qt, mAcc);

        op.angular_velocity.x = res.getX();
        op.angular_velocity.y = res.getY();
        op.angular_velocity.z = res.getZ();


        op.linear_acceleration.x = accel.getX();
        op.linear_acceleration.y = accel.getY();
        op.linear_acceleration.z = accel.getZ();

        pub.publish(op);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kitti_tf");
    ros::NodeHandle nh;
    ros::Subscriber sub;    

    pub = nh.advertise<sensor_msgs::Imu>("/imu_transformed", 1);

    sub = nh.subscribe("/kitti/oxts/imu", 1, transformCallback);

    ros::spin();


    return 0;
}