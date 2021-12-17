#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

std_msgs::Float64MultiArray solution;
ros::Publisher pub;

void matrixCallback(const std_msgs::Float64MultiArrayConstPtr& msg)
{
    int row_size = msg->layout.dim[0].size;
    int col_size = msg->layout.dim[1].size;

    // Check if inverse can exist
    // Input matrix should be a square matrix
    if(row_size == col_size)
    {
        Eigen::MatrixXd data(row_size, col_size);
        for(int i=0; i<row_size; i++)
        {
            std::vector<float> curr;
            for(int j=0; j<col_size; j++)
            {
                data(i,j) = msg->data[i*col_size + j];
            }
        }

        // Determinant of the matrix should be non zero
        if(std::fabs(data.determinant()) > 0.0)
        {
            Eigen::MatrixXd inv = data.inverse();

            solution.layout = msg->layout;
            solution.data.clear();

            std::vector<float> vec(row_size*col_size, 0);
            for(int i=0; i<row_size; i++)
            {
                for(int j=0; j<col_size; j++)
                {
                    vec[i*col_size + j] = inv(i,j);
                }
            }

            for(int i=0; i<vec.size(); i++)
            {
                solution.data.push_back(vec[i]);
            }
            pub.publish(solution);
        }
        else
        {
            ROS_WARN("Determinant of Matrix is zero! \n");
        }
    }
    else
    {
        ROS_WARN("Matrix is not a square matrix \n");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "subscribe_matrix");
    ros::NodeHandle n;
    pub = n.advertise<std_msgs::Float64MultiArray>("/inverse_publish", 1000);
    ros::Subscriber sub = n.subscribe("/matrix_publish", 1000, matrixCallback);


    ros::spin();
    return 0;

}