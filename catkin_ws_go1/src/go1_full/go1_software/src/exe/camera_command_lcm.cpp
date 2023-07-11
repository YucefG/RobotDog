/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>

#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include "convert.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    // Custom(uint8_t level): safe(LeggedType::A1), udp(level){
    //Custom(uint8_t level): safe(LeggedType::A1), udp(8090, "192.168.123.11", 8082, sizeof(HighCmd), sizeof(HighState)){
    //  Custom(uint8_t level): safe(LeggedType::A1), udp(8090, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState)){
    Custom(uint8_t level): safe(LeggedType::Go1), udp(8090, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState)){
    // Custom(uint8_t level): safe(LeggedType::Go1), udp(8090, "192.168.12.1", 8082, sizeof(HighCmd), sizeof(HighState)){
        udp.InitCmdData(cmd);
        // udp.SetDisconnectTime(dt, 1);
        // udp.SetDisconnectTime(0, 0);
    }
    void UDPRecv();
    void UDPSend();
    void RobotPosInit(void);
    void RobotCamControl(const geometry_msgs::Twist& twist);
    void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void ROSInit();
    //

    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    unitree_legged_msgs::HighCmd SendHighROS;
    unitree_legged_msgs::HighState RecvHighROS;
    
    //added
    ros::Subscriber sub;
    ros::NodeHandle nh;
    //

    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01
};

// added functions: 

//void Custom::callback(const unitree_legged_msgs::HighCmd::ConstPtr& msg)
//{
//    SendHighROS = *msg;
//}

//right one
void Custom::RobotCamControl(const geometry_msgs::Twist& twist)
{
    // move forward
    if(twist.linear.x > 0 && twist.angular.z == 0){
        cmd.mode = 2;
        cmd.gaitType = 1;
        cmd.velocity[0] = 0.2f; // -1  ~ +1
        //cmd.bodyHeight = 0.1;
        std::cout<<"forward"<<std::endl;
    }
    // move backward
    else if(twist.linear.x < 0 && twist.angular.z == 0){
        cmd.mode = 2;
        cmd.gaitType = 1;
        cmd.velocity[0] = -0.2f; // -1  ~ +1
        //cmd.bodyHeight = 0.1;
        std::cout<<"backward"<<std::endl;
    }
    //turn left (follow the facing arm)
    else if(twist.linear.x == 0 && twist.angular.z > 0){
        cmd.mode = 2;
        cmd.gaitType = 1;
        cmd.velocity[1] = 0.2f; 
        std::cout<<"left"<<std::endl;
    }
    else if(twist.linear.x == 0 && twist.angular.z < 0){
        cmd.mode = 2;
        cmd.gaitType = 1;
        cmd.velocity[1] = -0.2f; 
        std::cout<<"right"<<std::endl;

    }
    else{
        cmd.mode = 0;      // 0:idle, default stand      1:forced stand     2:walk continuously
        cmd.gaitType = 0;
        cmd.speedLevel = 0;
        cmd.footRaiseHeight = 0;
        cmd.bodyHeight = 0;
        cmd.euler[0]  = 0;
        cmd.euler[1] = 0;
        cmd.euler[2] = 0;
        cmd.velocity[0] = 0.0f;
        cmd.velocity[1] = 0.0f;
        cmd.yawSpeed = 0.0f;
    }

    udp.SetSend(cmd);
}

void Custom::twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    Custom::RobotCamControl(*msg);
}

void Custom::ROSInit()
{
    sub = nh.subscribe("/vel_cmd", 10, &Custom::twistCallback, this);
}

// end added functions


void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{  
    //added 
    // Update SendHighROS with new data from ROS subscriber
    ros::spinOnce();
    
    //udp.SetSend(SendHighROS);

    // end added

    udp.Send();
}

void Custom::RobotPosInit(void)
{
    cmd.mode = 0;    
    cmd.gaitType = 0;
    cmd.speedLevel = 0;
    cmd.footRaiseHeight = 0;
    cmd.bodyHeight = 0;
    cmd.euler[0]  = 0;
    cmd.euler[1] = 0;
    cmd.euler[2] = 0;
    cmd.velocity[0] = 0.0f;
    cmd.velocity[1] = 0.0f;
    cmd.yawSpeed = 0.0f;
}



int main(int argc, char **argv) 
{

    // initialize ROS
    ros::init(argc, argv, "hardware_high_cmd_node");

    std::cout << "Communication level is set to HIGH-level." << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();


    Custom custom(HIGHLEVEL);
    InitEnvironment();

    // init robot position
    custom.RobotPosInit();

    // create a node handle
    ros::NodeHandle nh;

    // subscribe to the "/cmd_vel" topic and specify the callback function
    ros::Subscriber sub = nh.subscribe("/cmd_vel", 1000, &Custom::twistCallback, &custom);

    // spin the node and process the callbacks
 //   ros::spin();
    // loopfunc does the same as ros spin but gives more possibility on timing
   // LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::twistCallback, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
   // loop_control.start();

    while(1){
        sleep(10);
    };

    return 0; 
}