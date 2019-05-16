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
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32MultiArray.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include "WPM2_Driver.h"
#include <math.h>
#include <vector>

using namespace std;

typedef struct
{
    float position[7];
    int velocity[7];
}st_wpm_pose;

static CWPM2_Driver wpv4_pt;
static double fDegToAng = 3.1415926/180;
static double fAngToDeg = 180/3.1415926;
static double pos_send[7];
static int vel_send[7];
static st_wpm_pose tmpPose;
static vector<st_wpm_pose> arPose;
static int nExecIndex = 0;

bool poseArrived()
{
    bool bArrived = true;
    for(int i=0;i<6;i++)
    {
        double fDegDiff = fabs(arPose[nExecIndex].position[i] - wpv4_pt.nRecvJointPos[i]*0.01);
        if(fDegDiff > 1)
        {
            bArrived = false;
        }
    }
    return bArrived;
}

//角度控制云台
void JointCtrlDegreeCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    int nNumJoint = msg->position.size();
    if(nNumJoint > 7)
    {
        nNumJoint = 7;
    }
    for(int i=0;i<nNumJoint;i++)
    {
        pos_send[i] = msg->position[i];
        vel_send[i] = msg->velocity[i];
        //ROS_INFO("[PT_JointCtrlDegreeCallback] %d - %s = %.2f", i, msg->name[i].c_str(),msg->position[i]);
    }
    wpv4_pt.SetJoints(pos_send,vel_send);
}

//弧度控制云台
void JointCtrlRadianCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    int nNumJoint = msg->position.size();
    if(nNumJoint > 7)
    {
        nNumJoint = 7;
    }
    for(int i=0;i<nNumJoint;i++)
    {
        if(i != 6)
        pos_send[i] = msg->position[i] * fAngToDeg;
        else
        pos_send[i] = msg->position[i]; //手爪
        vel_send[i] = msg->velocity[i];
        //ROS_INFO("[PT_JointCtrlRadianCallback] %d - %s = %.2f (%.0f Deg)", i, msg->name[i].c_str(),msg->position[i],pos_send[i]);
    }
    wpv4_pt.SetJoints(pos_send,vel_send);
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"wpv4_pt");
    ros::NodeHandle n;

    ros::Subscriber joint_ctrl_degree_sub = n.subscribe("/wpv4_pt/joint_ctrl_degree",30,&JointCtrlDegreeCallback);
    ros::Subscriber joint_ctrl_radian_sub = n.subscribe("/wpv4_pt/joint_ctrl_radian",30,&JointCtrlRadianCallback);

    ros::NodeHandle n_param("~");
    std::string strSerialPort;
    n_param.param<std::string>("serial_port", strSerialPort, "/dev/wpv4_pt");
    wpv4_pt.Open(strSerialPort.c_str(),115200);

    ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("/joint_states",100);
    sensor_msgs::JointState msg;
    std::vector<std::string> joint_name(2);
    std::vector<double> joint_pos(2);

    joint_name[0] = "wp_tilt";
    joint_name[1] = "wp_pitch";
    joint_pos[0] = 0.0f;
    joint_pos[1] = 0.0f;
    
    ros::Rate r(30);

    r.sleep();
    for(int i=0;i<7;i++)
    {
        pos_send[i] = 0;
        vel_send[i] = 2000;
    }
    wpv4_pt.SetJoints(pos_send,vel_send);
    int nCount = 0;

    while(n.ok())
    {
        wpv4_pt.ReadNewData();
        //ROS_INFO("[wpv4_pt.nParseCount]= %d",wpv4_pt.nParseCount);
        
        msg.header.stamp = ros::Time::now();
        msg.header.seq ++;

        double fTmp = 0;
        for(int i=0;i<2;i++)
        {
            fTmp = wpv4_pt.nRecvJointPos[i];
            fTmp *= 0.01;
            joint_pos[i] = fTmp*fDegToAng;
        }

        msg.name = joint_name;
        msg.position = joint_pos;
        joint_state_pub.publish(msg);

        ros::spinOnce();
        r.sleep();
    }
}