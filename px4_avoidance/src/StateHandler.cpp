#include "StateHandler.h"


StateHandler::StateHandler(std::string stateTopicName , ros::NodeHandle& node)
{
  
  stateSub = node.subscribe(stateTopicName , QUEUE_SIZE , &StateHandler::stateCallb , this);
  
}

StateHandler::~StateHandler()
{
  // work to do 
  ROS_INFO("closing connection to autopilot");
}



void StateHandler::stateCallb(const mavros_msgs::State::ConstPtr &_msg)
{
  
  state = *_msg;
  
}

bool StateHandler::isConnected()
{
  
  return this->state.connected;
  
}

void StateHandler::Connect(ros::Rate& rate)
{
  
  ROS_INFO("connecting to MAVROS ");
  
  while(ros::ok() && !this->isConnected()){
      
    ros::spinOnce();
    
    rate.sleep();
  }
  
  ROS_INFO("MAVROS connection established");
}