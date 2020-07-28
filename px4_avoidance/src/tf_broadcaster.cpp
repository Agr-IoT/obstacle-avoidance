#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>



nav_msgs::Odometry odom_frame;

void odom_callb(const nav_msgs::Odometry::ConstPtr odom)
{

    odom_frame.pose.pose.position.x = odom->pose.pose.position.x;
    odom_frame.pose.pose.position.y = odom->pose.pose.position.y;
    odom_frame.pose.pose.position.z = odom->pose.pose.position.z;

    odom_frame.pose.pose.orientation.w = odom->pose.pose.orientation.w;
    odom_frame.pose.pose.orientation.x = odom->pose.pose.orientation.x;
    odom_frame.pose.pose.orientation.y = odom->pose.pose.orientation.y;
    odom_frame.pose.pose.orientation.z = odom->pose.pose.orientation.z;

}


int main(int argc , char** argv)
{

    ros::init(argc , argv , "iris_tf_broadcaster");

    ros::NodeHandle node;
    

    tf::TransformBroadcaster br;
    tf::Transform transform;

    ros::Rate rate(1000.0);

    ros::Subscriber odom_sub = node.subscribe<nav_msgs::Odometry>("mavros/local_position/odom" , 30 , odom_callb);

    while(node.ok()){
        
        transform.setOrigin(tf::Vector3(odom_frame.pose.pose.position.x , odom_frame.pose.pose.position.y , odom_frame.pose.pose.position.z));
        transform.setRotation( tf::Quaternion(odom_frame.pose.pose.orientation.x , odom_frame.pose.pose.orientation.y, odom_frame.pose.pose.orientation.z , odom_frame.pose.pose.orientation.w ) );
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));

        rate.sleep();
        ros::spinOnce();
    }
    return 0;

}