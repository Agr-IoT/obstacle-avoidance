#include <ros/ros.h>
#include <std_msgs/String.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>

#include <local_planner/VfhPlus.h>


#define NODE_NAME "vfh_plus_planner"

using namespace ros;

int main(int argc, char **argv)
{
  init(argc, argv, NODE_NAME);
  NodeHandle n("~/");

  double frequency = 40.0;
  //n.param("frequency", frequency, 100.0);
  ROS_INFO("Operator will run at %.2f Hz.", frequency);

  VfhPlus planner;
   
  Rate loopRate(frequency);
  while(ok())
  {
    spinOnce();
    planner.vfhPlusPlanner();
    loopRate.sleep();
    if(loopRate.cycleTime() > ros::Duration(1.0 / frequency))
      ROS_WARN("Missed desired rate of %.2f Hz! Loop actually took %.4f Hz",frequency, 1/loopRate.cycleTime().toSec());
    }

  return 0;   
}