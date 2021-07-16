#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <vector>
#include <iostream>

#define PI 3.14159265

using namespace std;

bool path_generator(const double& radius, nav_msgs::Path& path);
string frame_id;
double radius;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_generator");
    ros::NodeHandle nh("~");

    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/path", 1);

    ros::param::get("~frame_id", frame_id);
    ros::param::get("~radius", radius);

    cout << frame_id << " " << radius << endl;

    nav_msgs::Path path;
    

    while(!path_generator(radius, path)) {}
    ros::Rate r(10);
    while(ros::ok())
    {
        path_pub.publish(path);
        ros::spinOnce();
        r.sleep();
    }
}

bool path_generator(const double& radius, nav_msgs::Path& path)
{
    // Generate the circle path
    double Ts = 0.1;

    double x0 = 0;
    double y0 = 0;

    path.header.frame_id = frame_id;
    path.header.stamp = ros::Time::now();
    for (int i = 0; i < 400; i++)
    {
        double x = radius*sin(PI/4 * i * Ts * 0.2);
        double y = radius*cos(PI/4 * i * Ts * 0.2);

        geometry_msgs::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.orientation.w = 1.0;

        path.poses.push_back(pose);
    }
    return true;
}