#ifndef A_STAR
#define A_STAR

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <mavros_msgs/Trajectory.h>
#include <cmath>
#include <string>
#include <vector>

#define PI 3.14159265

class AStar
{
public:
	// Default Constructor & Destructor
	 AStar ();
	~AStar ();
	
	
	void AStarPlanner();
private:
	void get_vector_magnitude();
	void get_certainity_grid();
	void build_polar_histogram();
	



	double _a = 1.4901;
	double _b = 0.0002;
	double alpha = 5 * (PI / 180.0);
	int k 		 = 72;

	// Internal Storage
	costmap_2d::Costmap2DROS* mLocalMap;
	costmap_2d::Costmap2D* mCostmap;
	
	
	tf::TransformListener mTfListener;
	tf2_ros::Buffer mTf2Buffer;
	tf2_ros::TransformListener mTf2Listener;
	
	
	int w_s = 100;
	std::string mOdometryFrame;
	std::string mRobotFrame;
	int  _certainity_grid  [100][100] = {0};
	int  		  _m_grid  [100][100] = {0};
	int           _b_direc [100][100] = {0};
	
	
	
};

#endif