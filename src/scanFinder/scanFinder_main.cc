#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	int rangeSize = scan->ranges.size();
	ROS_INFO("Range array size = %i", rangeSize);
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/scan", 10, chatterCallback);

	ros::Rate loop_rate(10.0);
	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
