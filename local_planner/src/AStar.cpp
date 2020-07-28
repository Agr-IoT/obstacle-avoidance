#include <nav_msgs/GridCells.h>
#include <math.h>

#include <local_planner/AStar.h>



#define WIDTH 100
#define HEIGHT 100

AStar ::AStar () : mTf2Buffer(), mTf2Listener(mTf2Buffer)
{
	// Create the local costmap
	mLocalMap = new costmap_2d::Costmap2DROS("local_map", mTf2Buffer);
	
	
	// Publish / subscribe to ROS topics
	ros::NodeHandle robotNode;
	robotNode.param("robot_frame", mRobotFrame, std::string("base_link"));
	robotNode.param("odometry_frame", mOdometryFrame, std::string("odom"));

	mRobotFrame = mTfListener.resolve(mRobotFrame);
	mOdometryFrame = mTfListener.resolve(mOdometryFrame);


}

AStar ::~AStar ()
{
		ROS_INFO("DESTROYING MAPPER");
} 


void AStar::get_certainity_grid()
{
	int p = 0;
	ROS_WARN("mapping ceartinity grid");
	for(int i = 0 ;i < WIDTH ; ++i){
		for(int j = 0 ; j < HEIGHT ; ++j){
			int l = static_cast<int>(mCostmap->getCost(i , j));

			double d_i_j = sqrt((49 - i)*(49 - i) + (49 - j)*(49 - j));
			if (int(d_i_j) < 6){
				++p;
				continue;
			}
			if (l > 0)
				_certainity_grid[i][j] += 1;
				continue;
			if (l = 0){
				if (_certainity_grid[i][j] == 0)
					continue;
				else if (_certainity_grid[i][j] > 0)
				{
					_certainity_grid[i][j] -= 1;
				}
				
			}
		}

	}

	ROS_WARN("end mapping ceartinity grid");
	ROS_WARN("%d " , p);
}

void AStar::get_vector_magnitude()
{


	for(int i = 0 ;i < WIDTH ; ++i){
		for(int j = 0 ; j < HEIGHT ; ++j){

			if (int(sqrt((49 - i)*(49 - i) + (49 - j)*(49 - j))) < 6){
				
				continue;
			}
			double d_i_j =  sqrt((49 - i)*(49 - i) + (49 - j)*(49 - j));
			_m_grid[i][j] = (_certainity_grid[i][j] * _certainity_grid[i][j])*(_a - _b * (d_i_j));


		}
	}

}

void AStar::build_polar_histogram()
{



}



void AStar ::AStarPlanner()
{
	// 1. Get a copy of the costmap to work on.
	mCostmap = mLocalMap->getCostmap();
	mLocalMap->updateMap();
	boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(mCostmap->getMutex()));
	int p = 0;
	ROS_WARN("mapping ceartinity grid");
	for(int i = 0 ;i < WIDTH ; ++i){
		for(int j = 0 ; j < HEIGHT ; ++j){
			int l = static_cast<int>(mCostmap->getCost(i , j));
			if (l > 0){
				ROS_WARN("cost is %d %d %d" , l , i , j);
				p++;
			}
			else
				continue;
		
			}
		}
	ROS_WARN("%d" , p);
}



double diff(double v1, double v2)
{
	if(v1 > v2)
		return v1 - v2;
	else
		return v2 - v1;
}


