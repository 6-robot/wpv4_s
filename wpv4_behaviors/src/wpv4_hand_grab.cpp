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
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <wpv4_behaviors/Coord.h>
#include <math.h>

// 抓取参数调节（单位：米）
static float grab_y_offset = 0.0f;                  //机器人对准物品，横向位移偏移量

static float target_dist = 0.85;                       //伸出手臂抓取前，对准物品的距离
static float target_x_k = 0.5;                          //对准物品时，前后移动的速度系数
static float target_z_k = 5.0;                          //对准物品时，旋转的速度系数

static ros::Publisher obj_track_pub;
static geometry_msgs::Pose obj_track_msg;
static ros::Publisher vel_pub;
static ros::Publisher mani_ctrl_pub;
static sensor_msgs::JointState mani_ctrl_msg;
static ros::Publisher grab_result_pub;
static std_msgs::String grab_result_msg;
static ros::Publisher palm_ctrl_pub;
static sensor_msgs::JointState palm_ctrl_msg;

static float obj_track_x = 0.0;
static float obj_track_y = 0.0;
static float obj_track_z = 0.0;

#define STEP_READY                     0
#define STEP_TAKE_AIM              1
#define STEP_REACH_OUT         2
#define STEP_FORWARD              3
#define STEP_GRAB_OBJ            4
#define STEP_TAKE_OVER          5
#define STEP_BACKWARD           6
#define STEP_DONE                       7
int step = STEP_READY;

void VelCmd(float inVx , float inVy, float inTz)
{
    geometry_msgs::Twist vel_cmd;
    vel_cmd.linear.x = inVx;
    vel_cmd.linear.y = inVy;
    vel_cmd.angular.z = inTz;
    vel_pub.publish(vel_cmd);
}

void PalmAction(int inAction)
{
    // 0 松开手掌
    if(inAction == 0)
    {
        for(int i=0;i<5;i++)
        {
            palm_ctrl_msg.position[i] = 100;
        }
        palm_ctrl_msg.position[5] = 0;
    }
    // 1 握住手掌
    if(inAction == 1)
    {
        palm_ctrl_msg.position[0] = 100;
        palm_ctrl_msg.position[1] = 100;
        palm_ctrl_msg.position[2] = 100;
        palm_ctrl_msg.position[3] = 100;
        palm_ctrl_msg.position[4] = 100;
        palm_ctrl_msg.position[5] = 0;
    }
    palm_ctrl_pub.publish(palm_ctrl_msg);
}

void ObjCoordCB(const wpv4_behaviors::Coord::ConstPtr &msg)
{
    if(step == STEP_TAKE_AIM)
    {
        // 获取盒子检测结果
        obj_track_x = msg->x[0];
        obj_track_y = msg->y[0];
        obj_track_z = msg->z[0];
        ROS_INFO("[ObjCoordCB]  %s  (%.2f , %.2f , %.2f)",msg->name[0].c_str(),msg->x[0],msg->y[0],msg->z[0]);
    }
}

void GrabObjCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    // 目标物品的坐标
    obj_track_x = msg->position.x;
    obj_track_y = msg->position.y;
    obj_track_z = msg->position.z;
    obj_track_msg.position.x = obj_track_x;
    obj_track_msg.position.y = obj_track_y;
    obj_track_msg.position.z = obj_track_z;
    obj_track_pub.publish(obj_track_msg);
    ROS_WARN("[GrabObjCallback] x = %.2f y= %.2f ,z= %.2f " ,obj_track_x, obj_track_y, obj_track_z);
    step = STEP_TAKE_AIM;
    ROS_WARN("**step -> STEP_TAKE_AIM**");
    grab_result_msg.data = "Take aim";
    grab_result_pub.publish(grab_result_msg);
    PalmAction(0);
}

float VelFixed(float inValue, float inMin, float inMax)
{
    float retValue = inValue;
    if(retValue > inMax)
    {
        retValue = inMax;
    }
    if(retValue < inMin)
    {
        retValue = inMin;
    }
    return retValue;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpv4_hand_grab");

    ros::NodeHandle nh;
    ros::Publisher behaviors_pub = nh.advertise<std_msgs::String>("/wpv4/behaviors", 10);
    obj_track_pub =  nh.advertise<geometry_msgs::Pose>("/wpv4/obj_track", 10);
    ros::Subscriber obj_result_sub = nh.subscribe("/wpv4/objects_3d", 10 , ObjCoordCB);
    ros::Subscriber grab_sub = nh.subscribe("/wpv4/grab_obj", 10, GrabObjCallback);
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 30);
    mani_ctrl_pub = nh.advertise<sensor_msgs::JointState>("/wpm2/joint_ctrl_degree", 30);
    grab_result_pub = nh.advertise<std_msgs::String>("/wpv4/grab_result", 30);
    palm_ctrl_pub = nh.advertise<sensor_msgs::JointState>("/rh56df3/set_angles", 30);

    ros::NodeHandle nh_param("~");
    nh_param.getParam("grab/target_z_k", target_z_k);
    //ROS_WARN("[grab_obj] target_z_k = %f",target_z_k);

    tf::TransformListener tf_listener; 
    tf::StampedTransform ar_transform;

    ros::Time time = ros::Time(0);
    
    // 手臂角度初始化
    mani_ctrl_msg.name.resize(7);
    mani_ctrl_msg.position.resize(7);
    mani_ctrl_msg.velocity.resize(7);
    for(int i=0;i<7;i++)
    {
        mani_ctrl_msg.position[i] = 0;
        mani_ctrl_msg.velocity[i] = 1000;
    }
    mani_ctrl_msg.position[0] = -90;
    mani_ctrl_msg.position[1] = -90;
    mani_ctrl_msg.position[2] = 90;
    mani_ctrl_msg.position[3] = -90;
    mani_ctrl_msg.position[4] = 0;
    mani_ctrl_msg.position[5] = 0;
    mani_ctrl_msg.position[6] = 0;

    // 手掌关节初始化
    palm_ctrl_msg.name.resize(6);
    palm_ctrl_msg.position.resize(6);
    palm_ctrl_msg.velocity.resize(6);
    PalmAction(0);

    int nCount = 0;
    ros::Rate r(30);
    while(nh.ok())
    {
        // 对准目标盒子
        if(step == STEP_TAKE_AIM)
        {
            float diff_x = obj_track_x - target_dist;
            float diff_y = obj_track_y - grab_y_offset;

            float vx = VelFixed(diff_x * target_x_k, -0.5, 0.5);
            float vz = VelFixed(diff_y * target_z_k, -0.2, 0.2); 

            VelCmd(vx,0,vz);
            ROS_WARN("diff_y = %.3f  vz = %.2f",diff_y,vz);
            if(fabs(diff_x) < 0.02 && fabs(diff_y) < 0.03)
            {
                VelCmd(0,0,0);
                std_msgs::String behavior_msg;
                behavior_msg.data = "object_track stop";
                behaviors_pub.publish(behavior_msg);
                nCount = 0;
                step = STEP_REACH_OUT;
                ROS_WARN("**step -> STEP_REACH_OUT**");
                grab_result_msg.data = "Reach out";
                grab_result_pub.publish(grab_result_msg);
            }
        }

        // 伸手准备抓取
        if(step == STEP_REACH_OUT)
        {
            if(nCount == 0)
            {
                mani_ctrl_msg.position[0] = -10;          //根旋转关节
                mani_ctrl_msg.position[1] = -110;
                mani_ctrl_msg.position[2] = 90;
                mani_ctrl_msg.position[3] = 40;
                mani_ctrl_msg.position[4] = 90;
                mani_ctrl_msg.position[5] = 60;
                mani_ctrl_msg.position[6] = 0;
                mani_ctrl_pub.publish(mani_ctrl_msg);
            }
            mani_ctrl_pub.publish(mani_ctrl_msg);
            nCount ++;
            if(nCount > 6* 30)
            {
                nCount = 0;
                step = STEP_FORWARD;
                ROS_WARN("**step -> STEP_FORWARD**");
                grab_result_msg.data = "Forward";
                grab_result_pub.publish(grab_result_msg);
            }
        }

        // 前进进行抓取
        if(step == STEP_FORWARD)
        {
            //VelCmd(0.1,0,0);
            mani_ctrl_msg.position[0] = 0;          //根旋转关节
            mani_ctrl_pub.publish(mani_ctrl_msg);
            nCount ++;
            if(nCount > 1.0* 30)
            {
                nCount = 0;
                step = STEP_GRAB_OBJ;
                ROS_WARN("**step -> STEP_GRAB_OBJ**");
                grab_result_msg.data = "Grab";
                grab_result_pub.publish(grab_result_msg);
            }
        }

        // 闭合手爪
        if(step == STEP_GRAB_OBJ)
        {
            VelCmd(0,0,0);
            PalmAction(1);
            nCount ++;
            if(nCount > 5* 30)
            {
                nCount = 0;
                step = STEP_TAKE_OVER;
                ROS_WARN("**step -> STEP_TAKE_OVER**");
                grab_result_msg.data = "Take over";
                grab_result_pub.publish(grab_result_msg);
            }
        }

        // 收回手臂
        if(step == STEP_TAKE_OVER)
        {
            VelCmd(0,0,0);
            mani_ctrl_msg.position[0] = 0;          //根旋转关节
            mani_ctrl_msg.position[1] = -30;
            mani_ctrl_msg.position[2] = 90;
            mani_ctrl_msg.position[3] = 120;
            mani_ctrl_msg.position[4] = 90;
            mani_ctrl_msg.position[5] = 60;
            mani_ctrl_msg.position[6] = 0;
            mani_ctrl_pub.publish(mani_ctrl_msg);
            nCount ++;
            if(nCount > 3* 30)
            {
                nCount = 0;
                step = STEP_BACKWARD;
                ROS_WARN("**step -> STEP_BACKWARD**");
                grab_result_msg.data = "Backward";
                grab_result_pub.publish(grab_result_msg);
            }
        }

        // 后退
        if(step == STEP_BACKWARD)
        {
            VelCmd(-0.1,0,0);
            nCount ++;
            if(nCount > 1* 30)
            {
                nCount = 0;
                step = STEP_DONE;
                ROS_WARN("**step -> STEP_DONE**");
            }
        }

        // 结束
        if(step == STEP_DONE)
        {
            if(nCount < 5)
            {
                VelCmd(0,0,0);
                grab_result_msg.data = "done";
                grab_result_pub.publish(grab_result_msg);
            }
            nCount ++;
        }
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
