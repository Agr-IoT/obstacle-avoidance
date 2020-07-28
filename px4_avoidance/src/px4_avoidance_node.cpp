#include <ros/ros.h>
#include <Eigen/Core>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <laser_geometry/laser_geometry.h>
#include <mavros_msgs/Trajectory.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CompanionProcessStatus.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/publisher.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <std_msgs/String.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>


#include <sensor_msgs/point_cloud_conversion.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <iostream>
#include <string>
#include <rosbag/buffer.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>


mavros_msgs::State current_state;

geometry_msgs::PoseStamped loc_pose;

mavros_msgs::Trajectory desired_path;

nav_msgs::Path adpated_path;

mavros_msgs::CompanionProcessStatus heart_msg;

sensor_msgs::LaserScan laser;

laser_geometry::LaserProjection projector_;

nav_msgs::OccupancyGrid map_msg;

sensor_msgs::PointCloud2 cloud;
sensor_msgs::PointCloud pcloud;

void grid_callb(const nav_msgs::OccupancyGrid::ConstPtr &occupancy_msg)
{
    map_msg = *occupancy_msg;
}

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
    
    
}


void laser_cb(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{

    laser = *scan_in;
    projector_.projectLaser(*scan_in , cloud);            
   for (int i = 0 ; i < 360; ++i){
            if (scan_in->ranges[i] > scan_in->range_max ){
                laser.ranges[i] = scan_in->range_max - 0.1;}
            else if (scan_in->ranges[i] < scan_in->range_min)
                {laser.ranges[i] = scan_in->range_min + 0.1;}
            continue;
    }
    sensor_msgs::convertPointCloud2ToPointCloud(cloud , pcloud); 
}

void Traject_msg_Callback(const mavros_msgs::Trajectory::ConstPtr& msg)
{
        //mirror the desired trajectory
 
        desired_path.point_valid[0]       = true;

        desired_path.point_1.position.x   = msg->point_1.position.x;
        desired_path.point_1.position.y   = msg->point_1.position.y;
        desired_path.point_1.position.z   = msg->point_1.position.z;
        desired_path.point_1.velocity.x   = msg->point_1.velocity.x;
        desired_path.point_1.velocity.y   = msg->point_1.velocity.y;
        desired_path.point_1.velocity.z   = msg->point_1.velocity.z;

        desired_path.point_1.yaw          = msg->point_1.yaw;
        desired_path.point_1.yaw_rate     = msg->point_1.yaw_rate;
  
  

} 

void pose_callb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

    loc_pose = *msg;

}


int main(int argc, char **argv)
{


    ros::init(argc, argv, "avoidance_node");
    
    ros::NodeHandle nh;
    

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 5, state_cb);

    ros::Subscriber sub_laser = nh.subscribe<sensor_msgs::LaserScan>
            ("laser/scan" , 20 , laser_cb );
    
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose" , 10 , pose_callb);

    ros::Subscriber desired_trajectory = nh.subscribe<mavros_msgs::Trajectory>
            ("mavros/trajectory/desired" , 10 , Traject_msg_Callback);  

    ros::Subscriber map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/vfh_plus_planner/local_map/costmap" , 20 ,grid_callb);
    //Publishers to publish relevant messages to topics

    ros::Publisher generate_path = nh.advertise<mavros_msgs::Trajectory>
            ("mavros/trajectory/generated" , 10 );

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    ros::Publisher path = nh.advertise<nav_msgs::Path>
            ("mavros/trajectory/path" , 10);

    ros::Publisher heartbeat = nh.advertise<mavros_msgs::CompanionProcessStatus>
            ("mavros/companion_process/status" , 10);
    
    ros::Publisher map_pub   = nh.advertise<nav_msgs::OccupancyGrid>("projected_map" , 20);

    ros::Publisher pub_laser = nh.advertise<sensor_msgs::LaserScan>
            ("scan" , 30);
    
    ros::Publisher pub_pcloud = nh.advertise<sensor_msgs::PointCloud>
            ("pointcloud"  , 10);
   
    ros::Publisher pcl2_pub = nh.advertise<sensor_msgs::PointCloud2>
            ("cloud_in" , 10);
    //Service clients to request services

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/takeoff");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(30.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        
        ros::spinOnce();
        
        rate.sleep();
    }


    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "AUTO.MISSION";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;


    heart_msg.state = heart_msg.MAV_STATE_ACTIVE;
    heart_msg.component = heart_msg.MAV_COMP_ID_OBSTACLE_AVOIDANCE;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
       
       
        generate_path.publish(desired_path);
        heartbeat.publish(heart_msg);
        pub_laser.publish(laser);
        pcl2_pub.publish(cloud);
        pub_pcloud.publish(pcloud);
        map_pub.publish(map_msg);
        ros::spinOnce();
        rate.sleep();
    }

  

   
    

    return 0;
}









































