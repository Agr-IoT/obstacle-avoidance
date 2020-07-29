#include <ros/ros.h>
#include <std_msgs/String.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

#include <local_planner/AStar.h>


#define NODE_NAME "a_star_planner"

using namespace ros;

int main(int argc, char **argv)
{
	init(argc, argv, NODE_NAME);
	NodeHandle n("~/");

	double frequency = 30.0;
	//n.param("frequency", frequency, 100.0);
	ROS_INFO("Operator will run at %.2f Hz.", frequency);

	AStar planner;
	
	Rate loopRate(frequency);
	while(ok())
	{
			spinOnce();
			planner.AStarPlanner();
			loopRate.sleep();
			if(loopRate.cycleTime() > ros::Duration(1.0 / frequency))
					ROS_WARN("Missed desired rate of %.2f Hz! Loop actually took %.4f seconds!",frequency, loopRate.cycleTime().toSec());
	}
	return 0;   
}