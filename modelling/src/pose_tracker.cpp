#include "head.h"
#include <ar_track_alvar_msgs/AlvarMarkers.h>

int num = 500;
int num_index = 0;
double x_avg = 0.0;
double y_avg = 0.0;
double z_avg = 0.0;
double roll_avg = 0.0;
double pitch_avg = 0.0;
double yaw_avg = 0.0;

void writeFile(string filename, vector<double> data)
{
    ofstream file;
    file.open(filename.c_str());
    for (int i = 0; i < data.size(); i++)
    {
        file << data[i] <<"\n";
    }
    file.close();
}
// void readCallback(cont std_msg::)
vector<double> xs;
vector<double> ys;
vector<double> zs;
vector<double> rolls;
vector<double> pitchs;
vector<double> yaws;

void readCallback(ar_track_alvar_msgs::AlvarMarkers msg)
{
    double roll, pitch, yaw, x, y, z;
    if (!msg.markers.empty()) 
    {
        if (msg.markers[0].id == 4)
        {
            tf::Quaternion q(msg.markers[0].pose.pose.orientation.x, msg.markers[0].pose.pose.orientation.y, msg.markers[0].pose.pose.orientation.z, msg.markers[0].pose.pose.orientation.w);
            tf::Matrix3x3 m(q);
            m.getRPY(roll, pitch, yaw);
            x = msg.markers[0].pose.pose.position.x;
            y = msg.markers[0].pose.pose.position.y;
            z = msg.markers[0].pose.pose.position.z;
            xs.push_back(x);
            ys.push_back(y);
            zs.push_back(z);
            rolls.push_back(roll);
            pitchs.push_back(pitch);
            yaws.push_back(yaw);

            if (num_index < num)
            {
                x_avg += x;
                y_avg += y;
                z_avg += z;
                roll_avg += roll;
                pitch_avg += pitch;
                yaw_avg += yaw;
                num_index ++;
                printf("[ Iteration : %d ] \n", num_index);
                usleep(100);
            }
            else
            {
                x_avg = x_avg / (num+1);
                y_avg = y_avg / (num+1);
                z_avg = z_avg / (num+1);
                roll_avg = roll_avg / (num+1);
                pitch_avg = pitch_avg / (num+1);
                yaw_avg = yaw_avg / (num+1);
                Eigen::Affine3f transform;
                transform = pcl::getTransformation(x_avg, y_avg, z_avg, roll_avg, pitch_avg, yaw_avg);
                Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
                for (int i = 0; i < 3; i++)
                    for (int j = 0; j < 3; j++)
                        matrix(i, j) = transform(i, j);
                matrix(0, 3) = x_avg;
                matrix(1, 3) = y_avg;
                matrix(2, 3) = z_avg;
                cout <<matrix << endl;
                matrix = matrix.inverse().eval();
                // matrix(0, 3) += 0.025;
                cout << matrix << endl;
                ofstream file;
                string filename = "transform_matrix.txt";
                file.open(filename.c_str());
                for (int i = 0; i < 4; i++)
                {
                    for (int j = 0; j < 4; j++)
                    {
                        file << matrix(i, j) << " ";
                    }
                    file << "\n";
                }
                file.close();
                ros::shutdown();
            }
        }
    }
    // tf::Pose marker;
    // ROS_INFO("[ x: %f ] \n", msg.markers[0].pose.pose.postion.x);
    // ROS_INFO("[ y: %f ] \n", msg.markers[0].pose.pose.postion.y);
    // ROS_INFO("[ z: %f ] \n", msg.markers[0].pose.pose.postion.z);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_reader");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("ar_pose_marker", 1000, readCallback);
    ros::spin();
    writeFile("xs.txt", xs);
    writeFile("ys.txt", ys);
    writeFile("zs.txt", zs);
    writeFile("rolls.txt", rolls);
    writeFile("pitchs.txt", pitchs);
    writeFile("yaws.txt", yaws);
    return (0);
}