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
/* @author Zhang Wanjie                                             */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <wpv4_behaviors/Coord.h>
#include <sensor_msgs/JointState.h>

static ros::Publisher waypoint_pub;
static std_msgs::Float64 plane_height_msg;
static ros::Publisher grab_obj_pub;
static geometry_msgs::Pose grab_obj_msg;
static ros::Publisher behaviors_pub;

#define STEP_READY                     0
#define STEP_GOTO_WP1           1
#define STEP_OBJ_DETECT       2
#define STEP_GRAB_OBJ            3
#define STEP_GOTO_WP2           4
#define STEP_DONE                       5
int step = STEP_READY;
int deley_count = 0;

void ObjCoordCB(const wpv4_behaviors::Coord::ConstPtr &msg)
{
    if(step == STEP_OBJ_DETECT)
    {
        // 获取盒子检测结果
        int obj_num = msg->name.size();
        ROS_INFO("[OBJCoordCB] obj_num = %d",obj_num);
        for(int i = 0;i<obj_num;i++)
        {
            ROS_INFO("[OBJCoordCB]  %s  (%.2f , %.2f , %.2f)",msg->name[i].c_str(),msg->x[i],msg->y[i],msg->z[i]);
        }
        int grab_obj_index = 0;
        grab_obj_msg.position.x = msg->x[grab_obj_index];
        grab_obj_msg.position.y = msg->y[grab_obj_index];
        grab_obj_msg.position.z = msg->z[grab_obj_index];
        grab_obj_pub.publish(grab_obj_msg);
        step = STEP_GRAB_OBJ;
    }
}

void NaviResultCB(const std_msgs::String::ConstPtr &msg)
{
    if(step == STEP_GOTO_WP1)
    {
        if(msg->data == "done")
        {
            ROS_INFO("[NaviResultCB] Waypoint 1!");
            std_msgs::String behavior_msg;
            behavior_msg.data = "object_detect start";
            behaviors_pub.publish(behavior_msg);
            step = STEP_OBJ_DETECT;
        }
    }
     
    if(step == STEP_GOTO_WP2)
    {
        if(msg->data == "done")
        {
            ROS_INFO("[NaviResultCB] Waypoint 2!");
            deley_count = 0;
            std_msgs::String behavior_msg;
            behavior_msg.data = "let_go start";
            behaviors_pub.publish(behavior_msg);
            step = STEP_DONE;
        }
    }
}

void GrabResultCB(const std_msgs::String::ConstPtr &msg)
{
    if(step == STEP_GRAB_OBJ)
    {
        if(msg->data == "done")
        {
            ROS_INFO("[GrabResultCB] grab_box done!");
            std_msgs::String waypoint_msg;
            waypoint_msg.data = "2";
            waypoint_pub.publish(waypoint_msg);
            step = STEP_GOTO_WP2;
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpv4_mobile_manipulation");

    ros::NodeHandle n;
    behaviors_pub = n.advertise<std_msgs::String>("/wpv4/behaviors", 10);
    waypoint_pub = n.advertise<std_msgs::String>("/waterplus/navi_waypoint", 10);
    ros::Subscriber navi_res_sub = n.subscribe("/waterplus/navi_result", 10, NaviResultCB);
    ros::Subscriber box_result_sub = n.subscribe("/wpv4/objects_3d", 10 , ObjCoordCB);
    grab_obj_pub = n.advertise<geometry_msgs::Pose>("/wpv4/grab_obj", 10);
    ros::Subscriber grab_res_sub = n.subscribe("/wpv4/grab_result", 10, GrabResultCB);

    sleep(1);

    ros::Rate r(10);
    while(ros::ok())
    {
        if(step == STEP_READY)
        {
            std_msgs::String waypoint_msg;
            waypoint_msg.data = "1";
            waypoint_pub.publish(waypoint_msg);
            step = STEP_GOTO_WP1;
        }

        ros::spinOnce();
        r.sleep();
    }
    

    return 0;
}
