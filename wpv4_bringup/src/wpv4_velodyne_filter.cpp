/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2017-2020, Waterplus http://www.6-robot.com
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WaterPlus nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/*!******************************************************************
 @author     ZhangWanjie
 ********************************************************************/

#include <ros/ros.h>
#include <math.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/LaserScan.h>

static std::string pub_topic;
ros::Publisher scan_pub;
ros::Subscriber scan_sub;
sensor_msgs::LaserScan new_scan;

float magnitude_of_ray(float x, float y, float z)
{
  return sqrt(pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0));
}

void VelodyneCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ> pointCloudIn;
    pcl::fromROSMsg(*msg , pointCloudIn);

    int cloudSize = pointCloudIn.points.size();
    
    new_scan.header.stamp = msg->header.stamp;
    new_scan.header.frame_id = msg->header.frame_id;
    new_scan.angle_max = 2*M_PI;
    new_scan.angle_min = 0;
    new_scan.angle_increment = M_PI/180;
    new_scan.time_increment = 0.001;
    new_scan.range_min = 0.25;
    new_scan.range_max = 100;
    new_scan.ranges.resize(360);
    new_scan.intensities.resize(360);

    for(int i=0 ; i<360 ; i++)
    {
        new_scan.ranges[i] = new_scan.range_max + 1.0;
        new_scan.intensities[i] = 0.0;
    }

    ////////////////////
    int n = 0;
    for(int i=0;i<cloudSize;i++)
    {
        if(pointCloudIn.points[i].z < 0.1 && pointCloudIn.points[i].z > -0.1)
        {
            n++;
            // ROS_INFO("[i= %d] ( %.2f , %.2f , %.2f)", 
            //     i , 
            //     pointCloudIn.points[i].x, 
            //     pointCloudIn.points[i].y, 
            //     pointCloudIn.points[i].z);
            float range = magnitude_of_ray(pointCloudIn.points[i].x, pointCloudIn.points[i].y, pointCloudIn.points[i].z);
            if(pointCloudIn.points[i].y != 0 && pointCloudIn.points[i].x != 0)
            {
                float angle = atan(pointCloudIn.points[i].y/pointCloudIn.points[i].x);
                int degree = angle * 180/M_PI;
                if( pointCloudIn.points[i].x < 0 )
                    degree += 180;
                while(degree > 360)
                    degree -= 360;
                while(degree < 0)
                    degree += 360;
                new_scan.ranges[degree] = range;
                new_scan.intensities[degree] = 1.0;
            }
        }
    }
    //ROS_INFO("cloudSize = %d n = %d",cloudSize,n);
    ////////////////////
    scan_pub.publish(new_scan);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wpv4_velodyne_filter");
    ROS_WARN("wpv4_velodyne_filter start");

    ros::NodeHandle n_param("~");
    n_param.param<std::string>("pub_topic", pub_topic, "/scan");

    ros::NodeHandle nh;
    ros::Subscriber pc_sub = nh.subscribe("/velodyne_points", 10 , VelodyneCallback);
    scan_pub = nh.advertise<sensor_msgs::LaserScan>(pub_topic,1);

    ros::spin();

    return 0;
}