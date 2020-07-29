#include <nav_msgs/GridCells.h>
#include <math.h>

#include <local_planner/VfhPlus.h>

//returns the direction of a cell in the certainty grid
double retDirection(int i_1 , int j_1 , int i_2 , int j_2);


VfhPlus ::VfhPlus () : mTf2Buffer(), mTf2Listener(mTf2Buffer)
{
	ROS_WARN("path planning initiated");
	// Create the local costmap
	mLocalMap = new costmap_2d::Costmap2DROS("local_map", mTf2Buffer);
	
	dcp_x = (WIDTH  / 2) - 1;
	dcp_y = (HEIGHT / 2) - 1;
	
	// Publish / subscribe to ROS topics
	ros::NodeHandle VfhNode;
	VfhNode.param("robot_frame", mRobotFrame, std::string("base_link"));
	VfhNode.param("odometry_frame", mOdometryFrame, std::string("odom"));

	desiredTrajectorySub = VfhNode.subscribe<mavros_msgs::Trajectory>
													("mavros/trajectory/desired" , 10 , &VfhPlus::trajCallb , this );
	adaptedPathPub  = VfhNode.advertise<mavros_msgs::Trajectory>("mavros/trajectory/generated" , 10 );

	mRobotFrame = mTfListener.resolve(mRobotFrame);
	mOdometryFrame = mTfListener.resolve(mOdometryFrame);

	initSectors();
	ROS_WARN("ending vfh+ planner constructor");

}

VfhPlus ::~VfhPlus ()
{
	ROS_WARN("path planning terminated!");
} 


void VfhPlus::initSectors()
{
	for(int sector_number = 0 ; sector_number < k ; ++sector_number)
	{
		primaryPolarHistogram[sector_number].sectorNum = sector_number;
		primaryPolarHistogram[sector_number].minAngle  = alpha_deg * sector_number;
		primaryPolarHistogram[sector_number].maxAngle  = alpha_deg * (sector_number + 1);
	}
}

void VfhPlus::trajCallb(const mavros_msgs::Trajectory::ConstPtr &desiredPath)
{
	this->desiredPath.point_1.position.x   = desiredPath->point_1.position.x;
	this->desiredPath.point_1.position.y   = desiredPath->point_1.position.y;
	this->desiredPath.point_1.position.z   = desiredPath->point_1.position.z;
	this->desiredPath.point_1.velocity.x   = desiredPath->point_1.velocity.x;
	this->desiredPath.point_1.velocity.y   = desiredPath->point_1.velocity.y;
	this->desiredPath.point_1.velocity.z   = desiredPath->point_1.velocity.z;
	this->desiredPath.point_1.yaw          = desiredPath->point_1.yaw;
	this->desiredPath.point_1.yaw_rate     = desiredPath->point_1.yaw_rate;

	this->desiredPath.point_2.yaw					 = desiredPath->point_2.yaw;
	
}

void VfhPlus::buildCertainityGrid()
{
	int drone_cell_count = 0;
	
	for(int i = 0 ;i < WIDTH ; ++i){
		for(int j = 0 ; j < HEIGHT ; ++j){
			
			int index = HEIGHT * i + j;
			
			int cost_probability  = static_cast<int>(mCostmap->getCost(i , j));
			double scale = static_cast<double>(100/255.0);
			int probability =  round(cost_probability * scale);
			double d_i_j  = sqrt((dcp_x - i)*(dcp_x - i) + (dcp_y - j)*(dcp_y - j));
			
			histogramGrid[index].i = i;
			histogramGrid[index].j = j;
			histogramGrid[index].d_i_j = d_i_j;
			histogramGrid[index].vectorDir = retDirection(dcp_x , dcp_y  , i , j);
			if(d_i_j < 6){
				++drone_cell_count;
				continue;
			}
			else{
				if(probability == 0){
					if (histogramGrid[index].cv == 0){
						continue;
					}
					else if(histogramGrid[index].cv > 0){
						histogramGrid[index].cv -= 1;
					}	
				}
				else if(50 >= probability > 0){	
					histogramGrid[index].cv++;
					histogramGrid[index].cv = std::min(20 , histogramGrid[index].cv);
					continue;
				}
				else{
					histogramGrid[index].cv += 2;
					histogramGrid[index].cv = std::min(20 , histogramGrid[index].cv);
					continue;
				}	
			}
		}
	}	
}



void VfhPlus::getVectorMagnitude()
{
	int pc = 0;
	int yu = 0;
	int num_cells = WIDTH * HEIGHT;
	for(int cellIndex = 0 ; cellIndex < num_cells ; ++cellIndex){
		
		int &certainty_value = histogramGrid[cellIndex].cv;
		histogramGrid[cellIndex].vectorMagnitude = (certainty_value * certainty_value )*(_a - _b * (histogramGrid[cellIndex].d_i_j * histogramGrid[cellIndex].d_i_j));
		if(histogramGrid[cellIndex].cv > 0){
			histogramGrid[cellIndex].enlargementAngle = asin((drone_rad + min_dist) / histogramGrid[cellIndex].d_i_j);	
			yu++;
			continue;
		}
		else if(histogramGrid[cellIndex].cv == 0)
		{
			histogramGrid[cellIndex].enlargementAngle  = 0;
			continue;
		}
		
	}	
}

void VfhPlus::buildPrimaryPolarHistogram()
{

	int numCells = WIDTH * HEIGHT;
	for(int cellIndex = 0 ; cellIndex < numCells ; ++cellIndex)
	{
		double h_i_j;
		if(histogramGrid[cellIndex].vectorMagnitude  == 0){
			continue;
		}
		else if(histogramGrid[cellIndex].vectorMagnitude  > 0){
			double m_i_j = histogramGrid[cellIndex].vectorMagnitude ;
			double e_angle = histogramGrid[cellIndex].enlargementAngle * (180.0/PI) ;
			double a_degrees = histogramGrid[cellIndex].vectorDir * (180.0/PI);
			int angle_degrees = round(a_degrees);
			int e_degrees = round(e_angle);

				
			int sector = int(angle_degrees / alpha_deg);
			if((a_degrees + e_angle) >= (sector * alpha_deg) && (sector * alpha_deg) >= (a_degrees - e_angle)){
				h_i_j = 1;
				primaryPolarHistogram[sector].Hp += m_i_j * h_i_j;		
			}
			else{
				h_i_j = 0;
				primaryPolarHistogram[sector].Hp += m_i_j * h_i_j;	
			}
		}
	}
}


void VfhPlus::buildMaskedPolarHistogram()
{

	//first build the binary polar histogram
	double thresholdLow 	= 1000.0;
	double thresholdHigh  = 6000.0;
	for(int sector = 0 ; sector < k ; ++sector)
	{
		if(primaryPolarHistogram[sector].Hp > thresholdHigh){
			primaryPolarHistogram[sector].occupied = false;
			continue;
		}
		else if(primaryPolarHistogram[sector].Hp < thresholdLow){
			primaryPolarHistogram[sector].occupied = true;
			continue;
		}
		else
		{
			primaryPolarHistogram[sector].occupied = primaryPolarHistogram[sector - 1].occupied; 
		}
	}

	//build the masked polar histogram

	int valleysMax = 16;
	int t = 50;

}




void VfhPlus::resetPolarHistogram()
{
	for(auto &sector:primaryPolarHistogram){
		sector.Hp = 0;
	}
}

void VfhPlus::printPrimaryPolarHistogram()
{

	for(auto i : primaryPolarHistogram){
		ROS_WARN("sector %d has %lf is Hp value" ,i.sectorNum ,i.Hp);
	}


}

void VfhPlus ::vfhPlusPlanner()
{
  // 1. Get a copy of the costmap to work on.
  mCostmap = mLocalMap->getCostmap();
  mLocalMap->updateMap();
  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(mCostmap->getMutex()));

  buildCertainityGrid();
  getVectorMagnitude();
  buildPrimaryPolarHistogram();
  printPrimaryPolarHistogram();
	buildMaskedPolarHistogram();
  resetPolarHistogram();
}




double retDirection(int i_1 , int j_1 , int i_2 , int j_2)
{

  int dif_i = i_1 - i_2;
  int dif_j = j_1 - j_2;
  double dif_d_i = static_cast<double>(dif_i);
  double dif_d_j = static_cast<double>(dif_j);
  double right_angle = PI / 2;

  //cells in first quadrant
  if(dif_i < 0 && dif_j <= 0 ){
	  if(dif_i < 0 && dif_j == 0){
	    return 0.0;
	  }
	  else if(dif_i < 0 && dif_j < 0){
	    return atan(dif_d_j/dif_d_i);
	  }	
  }

  //cells in second quadrant
  else if(dif_i >= 0 && dif_j < 0){
	  if(dif_i == 0 && dif_j < 0){
	    return right_angle;
	  }
	  else if(dif_i > 0 && dif_j < 0){
	    double angle = atan(dif_d_j/dif_d_i);
	    double dir   = right_angle + (right_angle + angle);
	    return dir;
	  }
  }

  //cells in third quadrant
  else if(dif_i > 0 && dif_j >= 0){
		if(dif_i > 0 && dif_j == 0){
	  	return 2 * right_angle;
		}
		else if(dif_i > 0 && dif_j > 0){
	  	double angle = atan(dif_d_j/dif_d_i);
	  	double dir 	 = 2 * right_angle  +  angle ;
	  	return dir;
		}
  }

  //cells in fourth quadrant
  else if(dif_i <= 0 && dif_j > 0){
		if(dif_i == 0 && dif_j > 0){
	  	return 3 * right_angle;
		}
		else if(dif_i < 0 && dif_j > 0){
	  	double angle = atan(dif_d_j/dif_d_i);
	  	double dir   = 3 * right_angle + (right_angle + angle);
	  	return dir;
		}
  }

  else{
		return 0.0;
  }
}



