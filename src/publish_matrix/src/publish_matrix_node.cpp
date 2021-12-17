#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <random>
#include <math.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "publish_matrix");
    ros::NodeHandle n;
    ros::Rate loop_rate(20);
    ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("/matrix_publish", 1000);

    constexpr int FLOAT_MIN = 0;
    constexpr int FLOAT_MAX = 1;
    std::random_device rd;
    std::default_random_engine eng(rd());
    std::uniform_real_distribution<float> distr(FLOAT_MIN, FLOAT_MAX);

    while(ros::ok())
    {
        std_msgs::Float64MultiArray msg;
        std::vector< std::vector<float> > mat;

        int row = std::ceil(distr(eng)*10);
        int col = row;

        if(distr(eng) < 0.25)
        {
            col = std::ceil(distr(eng)*10);            
        }

        for(int i=0; i<row; i++)
        {
            std::vector<float> curr;
            for(int j=0; j<col; j++)
            {
                curr.push_back(distr(eng));
            }
            mat.push_back(curr);
        }
        
        msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        msg.layout.dim[0].label = "row";
        msg.layout.dim[1].label = "column";
        msg.layout.dim[0].size = row;
        msg.layout.dim[1].size = col;
        msg.layout.dim[0].stride = row*col;
        msg.layout.dim[1].stride = row;
        msg.layout.data_offset = 0;

        std::vector<float> vec(row*col,0);
        for(int i=0; i<row; i++)
        {
            for(int j=0; j<col; j++)
            {
                vec[i*col + j] = mat[i][j];
            }
        }

        for(int i=0; i<vec.size(); i++)
        {
            msg.data.push_back(vec[i]);
        }
        //msg.data = vec;

        pub.publish(msg);
        loop_rate.sleep();
    }
    return 0;
}