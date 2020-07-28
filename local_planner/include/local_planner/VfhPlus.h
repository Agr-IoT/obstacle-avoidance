#ifndef VFH_PLUS
#define VFH_PLUS

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <mavros_msgs/Trajectory.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cmath>
#include <string>
#include <vector>

#define PI 3.141592653589793238463
#define WIDTH 100
#define HEIGHT 100

struct Cell
{
	int    i;
	int    j;
	int    cv = 0;
	double d_i_j; 
	double vectorDir;
	double vectorMagnitude;
	double enlargementAngle;
};

struct Sector
{
	
	int    sectorNum;
	double minAngle;
	double maxAngle;
	double Hp;

};



class VfhPlus
{
public:
	// Default Constructor & Destructor
	 VfhPlus ();
	~VfhPlus ();
	
	
	void vfhPlusPlanner();

private:

	//void grid_callback(const nav_msgs::OccupancyGrid::ConstPtr &occupancy_grid_msg);



	void getVectorMagnitude();
	void buildCertainityGrid();
	void buildPrimaryPolarHistogram();
	void initSectors();
	void printPrimaryPolarHistogram();
	void resetPolarHistogram();

	int dcp_x;
	int dcp_y;

	double _a = 1.4901;
	double _b = 0.0002;
	double alpha_rad = 5 * (PI / 180.0);
	double alpha_deg = 5;
	int k 		 = 72;
	int w_s = 100;
	float drone_rad = 0.3;
	float min_dist  = 0.5;

	// Internal Storage
	costmap_2d::Costmap2DROS* mLocalMap;
	costmap_2d::Costmap2D* mCostmap;
	
	ros::Subscriber occupancy_grid_sub;
	ros::Publisher adapted_path_pub;

	


	tf::TransformListener mTfListener;
	tf2_ros::Buffer mTf2Buffer;
	tf2_ros::TransformListener mTf2Listener;
	
	
	
	std::string mOdometryFrame;
	std::string mRobotFrame;
	

	Cell histogramGrid[WIDTH * HEIGHT] = {0 , 0 , 0 , 0.0 , 0.0 , 0.0 , 0.0};
	Sector primaryPolarHistogram[72]   = {0 ,0.0 , 0.0 , 0.0};
	
	
	
};

#endif