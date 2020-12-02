#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    //ROS_INFO("Imu Seq: [%d]", msg->header.seq);
    //ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);

    static tf::TransformBroadcaster odom_broadcaster;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(0, 0, 0) );
    tf::Quaternion quat(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
    //printf("[imu_odometry_wpv4] Yaw= %f \n",tf::getYaw(q)*180/3.1415);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    printf("[imu_odometry_wpv4] Roll= %.2f  Pitch= %.2f  Yaw= %.2f \n",roll*180/3.1415, pitch*180/3.1415, yaw*180/3.1415);
    transform.setRotation(quat);
    odom_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "imu_base", "base_footprint"));
}

int main(int argc, char **argv)
{
  ros::init(argc,argv, "imu_odometry_wpv4"); 
  ROS_INFO("[imu_odometry_wpv4]");

  ros::NodeHandle n;
  //odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Subscriber sub = n.subscribe("imu/data", 1000, imuCallback);
  ros::spin();

  return 0;
}