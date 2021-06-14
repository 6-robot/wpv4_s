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
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <math.h>


static ros::Publisher vel_pub;
static ros::Publisher mani_ctrl_pub;
static sensor_msgs::JointState mani_ctrl_msg;
static ros::Publisher let_go_result_pub;
static std_msgs::String let_go_result_msg;
static ros::Publisher palm_ctrl_pub;
static sensor_msgs::JointState palm_ctrl_msg;

#define STEP_READY                     0
#define STEP_HAND_UP               1
#define STEP_GRIPER_LOOSE   2
#define STEP_RECOVER                3
#define STEP_DONE                       4
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

void BehaviorCB(const std_msgs::String::ConstPtr &msg)
{
    int nFindIndex = 0;
    nFindIndex = msg->data.find("let_go start");
    if( nFindIndex >= 0 )
    {
        ROS_WARN("[let_go start] ");
        step = STEP_HAND_UP;
        ROS_WARN("**step -> STEP_HAND_UP**");
    }

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpv4_let_go");

    ros::NodeHandle nh;
    ros::Publisher behaviors_pub = nh.advertise<std_msgs::String>("/wpv4/behaviors", 10);
    ros::Subscriber behaviors_sub = nh.subscribe("/wpv4/behaviors", 30, BehaviorCB);
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 30);
    mani_ctrl_pub = nh.advertise<sensor_msgs::JointState>("/wpm2/joint_ctrl_degree", 30);
    let_go_result_pub = nh.advertise<std_msgs::String>("/wpv4/let_go_result", 30);
    palm_ctrl_pub = nh.advertise<sensor_msgs::JointState>("/rh56df3/set_angles", 30);

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

    int nCount = 0;
    ros::Rate r(30);
    while(nh.ok())
    {
        // 抬起手臂
        if(step == STEP_HAND_UP)
        {
            VelCmd(0,0,0);
            mani_ctrl_msg.position[0] = 0;          //根旋转关节
            mani_ctrl_msg.position[1] = 0;
            mani_ctrl_msg.position[2] = 90;
            mani_ctrl_msg.position[3] = 90;
            mani_ctrl_msg.position[4] = 90;
            mani_ctrl_msg.position[5] = 0;
            mani_ctrl_msg.position[6] = 0;
            mani_ctrl_pub.publish(mani_ctrl_msg);
            nCount ++;
            if(nCount > 5* 30)
            {
                nCount = 0;
                step = STEP_GRIPER_LOOSE;
                ROS_WARN("**step -> STEP_GRIPER_LOOSE**");
            }
        }

        // 松开手爪
        if(step == STEP_GRIPER_LOOSE)
        {
            VelCmd(0,0,0);
            PalmAction(0);
            nCount ++;
            if(nCount > 3* 30)
            {
                nCount = 0;
                step = STEP_RECOVER;
                ROS_WARN("**step -> STEP_RECOVER**");
                let_go_result_msg.data = "Recover";
                let_go_result_pub.publish(let_go_result_msg);
            }
        }

        // 收回手臂
        if(step == STEP_RECOVER)
        {
            VelCmd(0,0,0);
            mani_ctrl_msg.position[0] = -90;
            mani_ctrl_msg.position[1] = -90;
            mani_ctrl_msg.position[2] = 90;
            mani_ctrl_msg.position[3] = -90;
            mani_ctrl_msg.position[4] = 0;
            mani_ctrl_msg.position[5] = 0;
            mani_ctrl_msg.position[6] = 0;
            mani_ctrl_pub.publish(mani_ctrl_msg);
            nCount ++;
            if(nCount > 5* 30)
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
                let_go_result_msg.data = "done";
                let_go_result_pub.publish(let_go_result_msg);
            }
            nCount ++;
        }
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
