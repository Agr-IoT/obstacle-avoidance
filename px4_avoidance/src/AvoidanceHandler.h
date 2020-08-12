#ifndef AVOIDANCE_HANDLER
#define AVOIDANCE_HANDLER

#include <ros/ros.h>
#include <mavros_msgs/Trajectory.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/CompanionProcessStatus.h>
#include <string>

#define QUEUE_SIZE 10

#define STATE_TOPIC "mavros/state"


class AvoidanceHandler 
{


public:
    AvoidanceHandler(ros::NodeHandle& node);
    ~AvoidanceHandler();

public:
    bool isEnabled();
    void publishPath();


private:

    std::string heartbeatTopic;
    std::string generatedTrajectory;
    std::string desiredTrajectory;
    std::string adaptedPathtopic;

    mavros_msgs::Trajectory desiredPath;
    mavros_msgs::Trajectory adaptedPath;
    mavros_msgs::CompanionProcessStatus heartBeat;

    mavros_msgs::Trajectory publishedPath;

    ros::Subscriber plannerSubscriber;
    ros::Subscriber desiredTrajSub;


    
    ros::Publisher  plannerPathPublisher;
    ros::Publisher heartBeatAc;
    ros::ServiceClient avoidanceClient;

    mavros_msgs::ParamGet comObsAvoid;    
    


private:
    //mirror back the desired trajectory if no avoidance is enabled
    void desiredTraj(const mavros_msgs::Trajectory::ConstPtr& _msg);

    //genertaed trajectory by the planner
    void generateTraj(const mavros_msgs::Trajectory::ConstPtr& _msg);
};


#endif