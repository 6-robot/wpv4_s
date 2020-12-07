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
#include <sensor_msgs/JointState.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpv4_mini_pt");

    ros::NodeHandle n;
    ros::Publisher mini_pt_pub = n.advertise<sensor_msgs::JointState>("/wpv4/mini_pt_degree", 10);

    //机械臂控制消息包
    sensor_msgs::JointState ctrl_msg;
    ctrl_msg.name.resize(2);
    ctrl_msg.position.resize(2);
    ctrl_msg.velocity.resize(2);
    //关节角度
    ctrl_msg.position[0] = 0;
    ctrl_msg.position[1] = 0;
    //关节运动速度
    ctrl_msg.velocity[0] = 6000;
    ctrl_msg.velocity[1] = 6000;

    sleep(1);

    int nCount = 0;
    ros::Rate r(0.2);  //频率0.2Hz,5秒一个周期
    
    while(ros::ok())
    {
        ROS_INFO("[wpv4_mini_pt] nCount = %d", nCount);
        switch(nCount)
        {
        case 0:
            ctrl_msg.position[0] = 0;
            ctrl_msg.position[1] = 0;
            break;
        case 1:
            ctrl_msg.position[0] = 10;
            ctrl_msg.position[1] = 0;
            break;
        case 2:
            ctrl_msg.position[0] = 0;
            ctrl_msg.position[1] = 0;
            break;
        case 3:
            ctrl_msg.position[0] = -10;
            ctrl_msg.position[1] = 0;
            break;
        case 4:
            ctrl_msg.position[0] = 0;
            ctrl_msg.position[1] = 0;
            break;
        case 5:
            ctrl_msg.position[0] = 0;
            ctrl_msg.position[1] = -10;
            break;
        case 6:
            ctrl_msg.position[0] = 0;
            ctrl_msg.position[1] = 10;
            break;
        }
        mini_pt_pub.publish(ctrl_msg);
        ros::spinOnce();
        r.sleep();
        nCount ++;
        if(nCount >3)
        {
            nCount = 0;
        }
    }

    return 0;
}