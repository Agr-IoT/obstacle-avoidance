#ifndef CERTAINTY_GRID
#define CERTAINTY_GRID

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <memory>
#include <array>



using namespace costmap_2d;

struct Cell
{
  int i;
  int j;
  int cv;
};

class CertaintyGrid
{

public:
    CertaintyGrid(ros::NodeHandle &node);
    ~CertaintyGrid();
    void update();

    inline float getDistCP(); //distance to map centerpoint
    inline float getVectorMagnitude(); //vector magnitude of  cell


private:
    double paramA = 1.4901;
    double paramB = 0.0002;
    
    tf2_ros::Buffer mTf2Buffer;
    std::unique_ptr<Costmap2DROS> localMap;
    std::unique_ptr<Cell[]> certaintyGrid;

    int gridWidth;
    int gridHeight;
    float resolution;
    float droneRad;

    int mCenterI;
    int mCenterJ;    

};





#endif