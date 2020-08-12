#include "AvoidanceHandler.h"



AvoidanceHandler::AvoidanceHandler(ros::NodeHandle& node)
{
    ROS_INFO("AVOIDANCE: system starting");
    comObsAvoid.request.param_id = "COM_OBS_AVOID";

    heartbeatTopic      = "mavros/companion_process/status";
    desiredTrajectory   = "mavros/trajectory/desired";
    generatedTrajectory = "vfh_planner/trajectory/generated";
    adaptedPathtopic    = "mavros/trajectory/generated";

    plannerSubscriber = node.subscribe(generatedTrajectory , QUEUE_SIZE , &AvoidanceHandler::generateTraj , this);
    desiredTrajSub  = node.subscribe(desiredTrajectory , QUEUE_SIZE , &AvoidanceHandler::desiredTraj , this);
    plannerPathPublisher  = node.advertise<mavros_msgs::Trajectory>(adaptedPathtopic , QUEUE_SIZE);
    heartBeatAc    = node.advertise<mavros_msgs::CompanionProcessStatus>(heartbeatTopic , QUEUE_SIZE);
    avoidanceClient = node.serviceClient<mavros_msgs::ParamGet>("mavros/param/get");
    
}

bool AvoidanceHandler::isEnabled()
{
    if (avoidanceClient.call(comObsAvoid) && comObsAvoid.response.value.integer == 1)
        return true;
    return false;
}


void AvoidanceHandler::generateTraj(const mavros_msgs::Trajectory::ConstPtr& msg)
{
    if ( msg == NULL ) {
        return;
    }
    adaptedPath.point_valid[0]       = true;

    adaptedPath.point_1.position.x = msg->point_1.position.x;
    adaptedPath.point_1.position.y   = msg->point_1.position.y;
    adaptedPath.point_1.position.z   = 5;
    adaptedPath.point_1.velocity.x   = NAN;
    adaptedPath.point_1.velocity.y   = NAN;
    adaptedPath.point_1.velocity.z   = NAN;
  
    adaptedPath.point_1.acceleration_or_force.x   = NAN;
    adaptedPath.point_1.acceleration_or_force.y   = NAN;
    adaptedPath.point_1.acceleration_or_force.z   = NAN;
    adaptedPath.point_1.yaw          = msg->point_1.yaw;
    adaptedPath.point_1.yaw_rate     = msg->point_1.yaw_rate;
}

void AvoidanceHandler::desiredTraj(const mavros_msgs::Trajectory::ConstPtr& _msg)
{
    if ( _msg == NULL ) {
        return;
    }
    desiredPath = *_msg;
    
}

void AvoidanceHandler::publishPath()
{
    
    if (desiredPath.point_valid[0] == true)
      publishedPath  = desiredPath;
    if (adaptedPath.point_valid[0] == true)
      publishedPath = adaptedPath;
    

    heartBeat.state = heartBeat.MAV_STATE_ACTIVE;
    heartBeat.component = heartBeat.MAV_COMP_ID_OBSTACLE_AVOIDANCE;
    heartBeatAc.publish(heartBeat);
    if (publishedPath.point_valid[0] == true)
        plannerPathPublisher.publish(publishedPath);

}


AvoidanceHandler::~AvoidanceHandler()
{

    ROS_INFO("terminating path planning");
}