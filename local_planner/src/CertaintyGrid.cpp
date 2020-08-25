#include "CeartaintyGrid.h"
#include <algorithm>


#define WIDTH  10
#define HEIGHT 10
#define RESOLUTION 0.1
#define DRONE_RAD 0.3

#define SCALE(X)\
    X * (100.0/255)

CertaintyGrid::CertaintyGrid(ros::NodeHandle &node):mTf2Buffer()
{

  int width,height;

  node.param<int>("width" , width , WIDTH);
  node.param<int>("width" ,height , HEIGHT);
  node.param<float>("resolution" , resolution , RESOLUTION);
  node.param<float>("robot_radius" , droneRad , DRONE_RAD);
  gridWidth = int(width/resolution);
  gridHeight = int(height/resolution);

  ROS_INFO("resolution: %f  width: %d height: %d" , resolution ,gridWidth , gridHeight);

  mCenterI = (gridWidth / 2) - 1;
  mCenterJ = (gridWidth / 2) - 1;

  localMap = std::make_unique<Costmap2DROS>("local_Map" , mTf2Buffer);

  certaintyGrid = std::make_unique<Cell[]>(gridWidth * gridHeight);

  Cell init{0 ,0 ,0};
  std::fill(certaintyGrid.get() , certaintyGrid.get() + (gridWidth * gridHeight) , init);

  ROS_INFO("occupancy grid created");
  update();

}

inline float CertaintyGrid::getDistCP(int i , int j)
{
  return sqrt((mCenterI - i)*(mCenterI - i) + (mCenterJ - j)*(mCenterJ - j));
}

void CertaintyGrid::update()
{
  localMap->updateMap();
  //localMap->getCostmap()->getMutex()
  int numCells = gridWidth * gridHeight;
  for(int i = 0 ; i<gridWidth ; ++i)
  {
    for(int j = 0 ; j<gridHeight ;++j)
    {
      int index = gridHeight * i + j;
      int cost = localMap->getCostmap()->getCost(i , j);
      if(cost == costmap_2d::NO_INFORMATION)
      cost = costmap_2d::FREE_SPACE;       //no information cells, are given free
      int probability = round(SCALE(cost));
      certaintyGrid[index].i = i;
      certaintyGrid[index].j = j;
      float dist = getDistCP(i , j);
      int rRad = int(droneRad * (1/resolution));

      if(dist <= rRad)
        continue;
      else
      {
        if(probability == 0 || probability < 10)
        {
          if (certaintyGrid[index].cv == 0)
            continue;
          else if(certaintyGrid[index].cv > 0)
            certaintyGrid[index].cv -= 1;
        }
        else if(50 >= probability && probability> 10)
        {
          certaintyGrid[index].cv++;
          certaintyGrid[index].cv = std::min(20 , certaintyGrid[index].cv);
          continue;
        }
        else
        {
          certaintyGrid[index].cv += 2;
          certaintyGrid[index].cv = std::min(20 , certaintyGrid[index].cv);
          continue;
        }
      }
    }
  }
}