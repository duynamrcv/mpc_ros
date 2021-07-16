#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <iostream>
#include <vector>

using namespace std;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test");
	ros::NodeHandle nh;

	ros::Publisher predict_pub = nh.advertise<nav_msgs::Path>("/predict_path", 10);
	ros::Rate r(10);
	int count = 0;
	while (ros::ok())
	{
		nav_msgs::Path predict_path;
		predict_path.header.frame_id = "/map";
		predict_path.header.stamp = ros::Time::now();
		for (int i = 0; i < 10; i++)
		{
			geometry_msgs::PoseStamped pose;
			pose.header = predict_path.header;
		
		}
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}