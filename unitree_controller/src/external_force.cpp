/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <sensor_msgs/Joy.h>

class teleForceCmd
{
public:
    teleForceCmd();
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void pubForce(double x, double y, double z);
private:
    double Fx, Fy, Fz;
    ros::NodeHandle n;
    ros::Publisher force_pub;
    ros::Subscriber joy_sub;
    geometry_msgs::Wrench Force;
};

teleForceCmd::teleForceCmd()
{
    Fx = 0;
    Fy = 0;
    Fz = 0;
    force_pub = n.advertise<geometry_msgs::Wrench>("/apply_force/trunk", 20);
    joy_sub = n.subscribe<sensor_msgs::Joy>("/joy", 10, &teleForceCmd::joyCallback, this);
    sleep(1);
    pubForce(Fx, Fy, Fz);
}

void teleForceCmd::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    // Joystick axes mapping
    Fx = joy->axes[4] * 220;  // Right stick vertical axis controls Fx
    Fy = joy->axes[3] * 220;  // Right stick horizontal axis controls Fy
    Fz = joy->axes[1] * 220;  // Left stick vertical axis controls Fz

    ROS_INFO("Fx:%3d   Fy:%3d   Fz:%3d", (int)Fx, (int)Fy, (int)Fz);

    pubForce(Fx, Fy, Fz);
}

void teleForceCmd::pubForce(double x, double y, double z)
{
    Force.force.x = Fx;
    Force.force.y = Fy;
    Force.force.z = Fz;
    force_pub.publish(Force);
    ros::spinOnce();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "external_force");
    teleForceCmd remote;
    ros::spin();
    return 0;
}