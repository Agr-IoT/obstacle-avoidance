#include <nav_msgs/GridCells.h>
#include <math.h>
#include <typeinfo>
#include <algorithm>
#include "VfhPlus.h"

#define DEG(X)\
    X*(180/PI)
#define RAD(X)\
    X*(PI/180)

//returns the direction of a cell in the certainty grid
double retCellDirection(int i_1 , int j_1 , int i_2 , int j_2);


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
  
  localPoseSub         = VfhNode.subscribe<geometry_msgs::PoseStamped>
                          ("mavros/local_position/pose" , 10 , &VfhPlus::poseCallb , this );

  adaptedPathPub  = VfhNode.advertise<mavros_msgs::Trajectory>("vfh_planner/trajectory/generated" , 20 );


  mapi = VfhNode.advertise<nav_msgs::OccupancyGrid>("mapi" , 10);
  mRobotFrame = mTfListener.resolve(mRobotFrame);
  mOdometryFrame = mTfListener.resolve(mOdometryFrame);

  initSectors();
  
  mLocalMap->resetLayers();
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

void VfhPlus::poseCallb(const geometry_msgs::PoseStamped::ConstPtr &_msg)
{

  this->localPosition = *_msg;
  
}

void VfhPlus::trajCallb(const mavros_msgs::Trajectory::ConstPtr &desiredPathf)
{
  this->desiredPath.point_valid[0] = true;
  this->desiredPath.point_1.position.x   = desiredPathf->point_1.position.x;
  this->desiredPath.point_1.position.y   = desiredPathf->point_1.position.y;
  this->desiredPath.point_1.position.z   = desiredPathf->point_1.position.z;
  this->desiredPath.point_1.velocity.x   = desiredPathf->point_1.velocity.x;
  this->desiredPath.point_1.velocity.y   = desiredPathf->point_1.velocity.y;
  this->desiredPath.point_1.velocity.z   = desiredPathf->point_1.velocity.z;
  this->desiredPath.point_1.yaw          = desiredPathf->point_2.yaw;
  this->desiredPath.point_1.yaw_rate     = desiredPathf->point_1.yaw_rate;

  this->desiredPath.point_2.yaw          = desiredPathf->point_2.yaw;
}

void VfhPlus::buildCertainityGrid()
{
  int drone_cell_count = 0;
  
  for(int i = 0 ;i < WIDTH ; ++i)
  {
    for(int j = 0 ; j < HEIGHT ; ++j)
    {
      
      int index = HEIGHT * i + j;
      int costProbability  = static_cast<int>(mCostmap->getCost(i , j));
      if(costProbability == 255)
      {
        costProbability = 0;
      }
      double scale = static_cast<double>(100/255.0);
      int probability =  round(costProbability * scale);
      double d_i_j  = sqrt((dcp_x - i)*(dcp_x - i) + (dcp_y - j)*(dcp_y - j));
      
      histogramGrid[index].i = i;
      histogramGrid[index].j = j;
      histogramGrid[index].d_i_j = d_i_j;
      histogramGrid[index].vectorDir = retCellDirection(dcp_x , dcp_y  , i , j);
      double der = atan2( j - dcp_y  , i - dcp_x);

      ROS_INFO("i->%d j->%d der->%lf  vdir->%lf" ,i , j ,  DEG(der) , DEG(histogramGrid[index].vectorDir));
      

      if(d_i_j < 6)
      {
        ++drone_cell_count;
        continue;
      }
      else
      {
        if(probability == 0 || probability < 10.0)
        {
          if (histogramGrid[index].cv == 0){
            continue;
          }
          else if(histogramGrid[index].cv > 0)
          {
            histogramGrid[index].cv -= 1;
          }	
        }
        else if(50 >= probability && probability> 10)
        {	
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
  for(int cellIndex = 0 ; cellIndex < num_cells ; ++cellIndex)
  
  {
    Cell &c = histogramGrid[cellIndex];
    int &certainty_value = histogramGrid[cellIndex].cv;
    
    histogramGrid[cellIndex].vectorMagnitude = (certainty_value * certainty_value )*(_a - _b * (histogramGrid[cellIndex].d_i_j * histogramGrid[cellIndex].d_i_j));
    if(histogramGrid[cellIndex].cv > 0)
    {
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
    if(histogramGrid[cellIndex].vectorMagnitude  == 0)
    {
      continue;
    }
    else if(histogramGrid[cellIndex].vectorMagnitude  > 0)
    {
      double m_i_j = histogramGrid[cellIndex].vectorMagnitude ;
      double e_angle = histogramGrid[cellIndex].enlargementAngle * (180.0/PI) ;
      double a_degrees = histogramGrid[cellIndex].vectorDir * (180.0/PI);
      int angle_degrees = round(a_degrees);
      int e_degrees = round(e_angle);

        
      int sector = int(angle_degrees / alpha_deg);
      if((a_degrees + e_angle) >= (sector * alpha_deg) && (sector * alpha_deg) >= (a_degrees - e_angle))
      {
        h_i_j = 1;
        primaryPolarHistogram[sector].Hp += m_i_j * h_i_j;		
      }
      else
      {
        h_i_j = 0;
        primaryPolarHistogram[sector].Hp += m_i_j * h_i_j;	
      }
    }
  }
}

void VfhPlus::binaryPolarHistogram()
{
  //first build the binary polar histogram
  double thresholdLow 	= 500.0;
  double thresholdHigh  = 1000.0;
  for(int sector = 0 ; sector < k ; ++sector)
  {
    if(primaryPolarHistogram[sector].Hp >= thresholdHigh)
    {
      primaryPolarHistogram[sector].occupied = 1;
      continue;
    }
    else if(primaryPolarHistogram[sector].Hp < thresholdLow)
    {
      primaryPolarHistogram[sector].occupied = 0;
      continue;
    }
    else
    {
      primaryPolarHistogram[sector].occupied = primaryPolarHistogram[sector - 1].occupied; 
    }
  }
}

void VfhPlus::buildMaskedPolarHistogram()
{

  //build the masked polar histogram

  int valleysMax = 16;
  int turningRadius = 25;
  double jB;
  double droneDirection = direction(desiredPath.point_2.yaw);
  jB = droneDirection + PI;
  
  double jR = jB;
  double jL = jB;
  
  
  double dxr =   turningRadius * std::sin(droneDirection);
  double dyr =   -1 * turningRadius * std::cos(droneDirection);

  double dxl = -1 *turningRadius * std::sin(droneDirection);
  double dyl = turningRadius * std::cos(droneDirection);
  
  
  for(int index = 0 ; index < (WIDTH * HEIGHT); ++index)
  {
    
    if(histogramGrid[index].cv == 0)
    {
      continue;
    }
    else if (histogramGrid[index].cv > 0)
    {
      Cell &c = histogramGrid[index];
      
 
      float dr = ((dxr - (c.i - dcp_x )) * (dxr - (c.i - dcp_x ))) + ((dyr - (c.j- dcp_y )) *(dyr - (c.j- dcp_y))); 
      float dl = ((dxl - (c.i - dcp_x )) * (dxl - (c.i- dcp_x ))) + ((dyl - (c.j- dcp_y )) *(dyl - (c.j- dcp_y ))); 
      float dbc = (turningRadius + (min_dist + drone_rad)*10) * (turningRadius + (min_dist + drone_rad)*10);
         
      if(dr <= dl)
      {
        double dif = std::fabs(droneDirection - c.vectorDir);
        
        if(dr <= dbc)
        {
          jR = c.vectorDir;
          continue;
        }
      }
      else if(dr > dl)
      {
        
        if(dl < dbc)
        {
          jL = c.vectorDir;
          
        }
      } 
    }
  }


  double q = droneDirection;
  for(auto &sector : primaryPolarHistogram)
  {
    double mAngle = sector.minAngle;
    double dif = std::fabs(droneDirection - mAngle);
    double difR = std::fabs(droneDirection - jR);
    double difL = std::fabs(droneDirection - jL);
    if((mAngle > jR) || (mAngle < jL))
    {
      if(sector.occupied == 0)
      {
        sector.occupiedMasked = 0;
      }
      else
      {
        sector.occupiedMasked = 1;
      }      
    } 
    else
    {
      sector.occupiedMasked = 1;
    }
  }
 
}


void VfhPlus::resetPolarHistogram()
{
  for(auto &sector:primaryPolarHistogram)
  {
    sector.Hp = 0;
  }
}

void VfhPlus::printPrimaryPolarHistogram()
{
  for(auto i : primaryPolarHistogram)
  {
    ROS_WARN("sector %d Hp->%lf A->%lf occu->%d " ,i.sectorNum ,i.Hp, i.minAngle , i.occupied);
    
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
  binaryPolarHistogram();
  selectValleys();
  executeCommand();
  adaptedPathPub.publish(adaptedPath);
  resetPolarHistogram();
  valleys.clear();
  
}

void VfhPlus::executeCommand()
{
  this->adaptedPath.point_valid[0] = true;
  
  this->adaptedPath.point_1.yaw_rate = this->desiredPath.point_1.yaw_rate;
  double dir;
  if(desiredPath.point_valid[0] == true)
    dir = DEG(direction(desiredPath.point_2.yaw));
  else
    return;
  double dif = dir + PI;
  double pubAng = dif;
  double cost;
  std::vector<double> acost;
  
  for(auto valley : valleys)
  {
    
    double mid = (valley.min_angle + valley.max_angle)/2;
    cost = std::fabs(dir - mid);
    
    if (cost <= dif)
    {
      pubAng = mid;
      dif = cost;
    }
    acost.push_back(cost);
    
    
  }

  float step_distance = 1.5; //step distance in meters in the direction with least cost
  
  std::vector<double>::iterator result = std::min_element(acost.begin(), acost.end());
  int i = std::distance(acost.begin(), result);
  pubAng = (valleys[i].min_angle + valleys[i].max_angle)/2;  
  this->adaptedPath.point_1.yaw = this->desiredPath.point_1.yaw;
  adaptedPath.point_2.yaw = RAD(pubAng);
  adaptedPath.point_1.position.x = step_distance* std::cos(RAD(pubAng)) + localPosition.pose.position.x;
  adaptedPath.point_1.position.y = step_distance * std::sin(RAD(pubAng)) + localPosition.pose.position.y;
  adaptedPath.point_1.position.z = 5;
  adaptedPath.point_1.velocity.x = NAN;
  adaptedPath.point_1.velocity.y = NAN;
  adaptedPath.point_1.velocity.z = NAN;
  adaptedPath.point_1.acceleration_or_force.x = NAN;
  adaptedPath.point_1.acceleration_or_force.y = NAN;
  adaptedPath.point_1.acceleration_or_force.z = NAN;
  
}

void VfhPlus::selectValleys()
{
  
  Valley valley{0.0 ,0.0 , 0};
  int j = 0; 
  int g = 0;
  for(int i = 0; i < k ;++i)
  {

    valley.min_angle = primaryPolarHistogram[g].minAngle;
    if(primaryPolarHistogram[i].occupied == 0)
    {
      valley.max_angle = primaryPolarHistogram[i].maxAngle;
      valley.secCount += 1; 
      if(valley.secCount == 8)
      {
        g = i + 1;
        Valley v = valley;
        valleys.push_back(v);
        valley.secCount = 0;
        continue;
      }
      if(valley.secCount < 5)
      {
        if(primaryPolarHistogram[i + 1].occupied == 1)
        {
          g = i + 2;
          Valley v = valley;
          valleys.push_back(v);
          valley.secCount = 0;
          continue;
        }
        else if(i == (k-1))
        {
          Valley v = valley;
          valleys.push_back(v);
          valley.secCount = 0;
          continue;
        } 
      }   
    }
    else if(primaryPolarHistogram[i].occupied == 1)
    {
      g++;
      continue;
    }
  }
}

double VfhPlus::direction(float angle)
{
  if(angle <= 0 && angle >= -(PI/2))
  {
    return 2*PI + angle;
  }

  else if (angle < -(PI/2) && angle >= -(PI))
  {
    double dir = PI/2 + angle;
    dir = (PI/2 + angle) + (3 * PI/2);
    return dir;
  }
  else
  {
    return angle;
  }
  
}


double retCellDirection(int i_1 , int j_1 , int i_2 , int j_2)
{

  int dif_i = i_1 - i_2;
  int dif_j = j_1 - j_2;
  double dif_d_i = static_cast<double>(dif_i);
  double dif_d_j = static_cast<double>(dif_j);
  double right_angle = PI / 2;

  //cells in first quadrant
  if(dif_i < 0 && dif_j <= 0 )
  {
    if(dif_i < 0 && dif_j == 0)
    {
      return 0.0;
    }
    else if(dif_i < 0 && dif_j < 0)
    {
      return atan(dif_d_j/dif_d_i);
    }	
  }

  //cells in second quadrant
  else if(dif_i >= 0 && dif_j < 0)
  {
    if(dif_i == 0 && dif_j < 0)
    {
      return right_angle;
    }
    else if(dif_i > 0 && dif_j < 0)
    {
      double angle = atan(dif_d_j/dif_d_i);
      double dir   = right_angle + (right_angle + angle);
      return dir;
    }
  }

  //cells in third quadrant
  else if(dif_i > 0 && dif_j >= 0)
  {
    if(dif_i > 0 && dif_j == 0)
    {
      return 2 * right_angle;
    }
    else if(dif_i > 0 && dif_j > 0)
    {
      double angle = atan(dif_d_j/dif_d_i);
      double dir 	 = 2 * right_angle  +  angle ;
      return dir;
    }
  }

  //cells in fourth quadrant
  else if(dif_i <= 0 && dif_j > 0)
  {
    if(dif_i == 0 && dif_j > 0)
    {
      return 3 * right_angle;
    }
    else if(dif_i < 0 && dif_j > 0)
    {
      double angle = atan(dif_d_j/dif_d_i);
      double dir   = 3 * right_angle + (right_angle + angle);
      return dir;
    }
  }

  else
  {
    return 0.0;
  }
}



