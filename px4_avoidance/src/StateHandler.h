#ifndef STATE_HANDLER
#define STATE_HANDLER

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <string>

#define QUEUE_SIZE 10



class StateHandler
{
public:
  StateHandler(std::string stateTopicName ,  ros::NodeHandle& node);
  ~StateHandler();
  
  void setState();
  void Connect(ros::Rate& rate);
  bool isConnected();
  std::string getState();

private:
   mavros_msgs::SetMode mode;
   mavros_msgs::State state;
   ros::Subscriber stateSub;
   ros::ServiceClient setModeClient;

private:
   void stateCallb(const mavros_msgs::State::ConstPtr& _msg);
};


#endif


