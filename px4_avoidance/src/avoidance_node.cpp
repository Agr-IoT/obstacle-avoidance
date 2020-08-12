
#include <ros/ros.h>
#include "StateHandler.h"
#include "AvoidanceHandler.h"
#include <string>

#define NODE_NAME "avoidance_node"
#define STATE_TOPIC "mavros/state"

int main(int argc , char** argv)
{

   ros::init(argc, argv, NODE_NAME);

   ros::NodeHandle node;
   ros::Rate rate(30.0);
   StateHandler fcuState(STATE_TOPIC , node);

   fcuState.Connect(rate);
   AvoidanceHandler avoidanceSystem(node);

   while(ros::ok()){
      avoidanceSystem.publishPath();
      ros::spinOnce();
      rate.sleep();
   }

   return 0;


}