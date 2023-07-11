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
#include <std_msgs/String.h>

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
    void velcommand_to_udp(const geometry_msgs::Twist& cmd_vel);
    void posecommand_to_udp(const geometry_msgs::Twist& cmd_pose);

    void velCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel);
    void modeCallback(const std_msgs::String::ConstPtr& audio_mode);
    void poseCallback(const geometry_msgs::Twist::ConstPtr& cmd_pose);



    void ROSInit();
    //

    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    unitree_legged_msgs::HighCmd SendHighROS;
    unitree_legged_msgs::HighState RecvHighROS;
    
    //added
    ros::Subscriber subVel;
    ros::Subscriber subPose;
    ros::Subscriber subMode;

    ros::NodeHandle nh;
    //

    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01

    string command_mode = "stop"; //among 'stop', 'find_human', 'vel_command', 'pose_command', ...
};

// added functions: 

//right one
void Custom::velcommand_to_udp(const geometry_msgs::Twist& cmd_vel)
{
    //std::cout<<"commandmode is "<<command_mode<<std::endl;

    if(command_mode=="undefined"){
        std::cout<<"in undefined"<<std::endl;
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
    else if(command_mode=="find_human")
    {
        std::cout<<"find human"<<std::endl;
        // move forward
        if(cmd_vel.linear.x > 0 && cmd_vel.angular.z == 0){
            cmd.mode = 2;
            cmd.gaitType = 1;
            cmd.velocity[0] = 0.2f; // -1  ~ +1
            //cmd.bodyHeight = 0.1;
            std::cout<<"forward"<<std::endl;
        }
        // move backward
        else if(cmd_vel.linear.x < 0 && cmd_vel.angular.z == 0){
            cmd.mode = 2;
            cmd.gaitType = 1;
            cmd.velocity[0] = -0.2f; // -1  ~ +1
            //cmd.bodyHeight = 0.1;
            std::cout<<"backward"<<std::endl;
        }
        //turn ON ITSELF left
        else if(cmd_vel.linear.x == 0 && cmd_vel.angular.z > 0){
            cmd.mode = 2;
            cmd.gaitType = 1;
            cmd.velocity[2] = 0.4f; 
            std::cout<<"left turning"<<std::endl;
        }
        else if(cmd_vel.linear.x == 0 && cmd_vel.angular.z < 0){
            cmd.mode = 2;
            cmd.gaitType = 1;
            cmd.velocity[2] = -0.4f; 
            std::cout<<"right turning"<<std::endl;
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
            std::cout<<"stopped"<<std::endl;
        }
    }
    else if(command_mode=="vel_command")
    {
        std::cout<<"human body command"<<std::endl;
        // move forward
   //     std::cout<<cmd_vel.linear.x<<" "<<cmd_vel.linear.y<<std::endl;

        if(cmd_vel.linear.x > 0 && cmd_vel.linear.y == 0){
            cmd.mode = 2;
            cmd.gaitType = 1;
            cmd.velocity[0] = 0.2f; // -1  ~ +1
            //cmd.bodyHeight = 0.1;
            std::cout<<"forward"<<std::endl;
        }
        // move backward
        else if(cmd_vel.linear.x < 0 && cmd_vel.linear.y  == 0){
            cmd.mode = 2;
            cmd.gaitType = 1;
            cmd.velocity[0] = -0.2f; // -1  ~ +1
            //cmd.bodyHeight = 0.1;
            std::cout<<"backward"<<std::endl;
        }
        //turn left (follow the facing arm)
        else if(cmd_vel.linear.x == 0 && cmd_vel.linear.y  > 0){
            cmd.mode = 2;
            cmd.gaitType = 1;
            cmd.velocity[1] = -0.2f; 
            std::cout<<"left lateral"<<std::endl;
        }
        else if(cmd_vel.linear.x == 0 && cmd_vel.linear.y < 0){
            cmd.mode = 2;
            cmd.gaitType = 1;
            cmd.velocity[1] = 0.2f; 
            std::cout<<"right lateral"<<std::endl;
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
            std::cout<<"stopped"<<std::endl;
        }
    }
    udp.SetSend(cmd);
}

void Custom::posecommand_to_udp(const geometry_msgs::Twist& cmd_pose)
{
    std::cout <<"command mode is: "<< command_mode<<std::endl;
    if(command_mode=="pose_command")
    {
        cmd.mode=1;
        cmd.euler[0] = cmd_pose.angular.x; //roll
        cmd.euler[1] = cmd_pose.angular.y; //pitch
        std::cout<<"roll angle given to udp: "<<cmd.euler[0]<<std::endl;
        std::cout<<"pitch angle given to udp: "<<cmd.euler[1]<<std::endl;
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
    std::cout<<cmd.mode<<std::endl;
    udp.SetSend(cmd);
}

void Custom::modeCallback(const std_msgs::String::ConstPtr& audio_mode)
{
    //if robot was controlled by tilting, make sure that it gets to initial body pose when following velocity commands
    if (command_mode=="pose_command" && audio_mode->data!="pose_command")
    {
        std::cout<<"ROBOT INIT !!!!"<<std::endl;
        RobotPosInit();
    }
    command_mode = audio_mode->data;
    //std::cout<<"audio mode is: "<<command_mode<<std::endl;

}

void Custom::poseCallback(const geometry_msgs::Twist::ConstPtr& cmd_pose)
{
    Custom::posecommand_to_udp(*cmd_pose);
}

void Custom::velCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
    Custom::velcommand_to_udp(*cmd_vel);
}

void Custom::ROSInit()
{
    subMode = nh.subscribe("/audio/mode", 10, &Custom::poseCallback, this);
    subVel = nh.subscribe("/vel_cmd", 10, &Custom::velCallback, this);
    subPose = nh.subscribe("/mode_cmd", 10, &Custom::modeCallback, this);
}


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
    std::cout << "INIT !!!!!!!" << std::endl<<std::endl;
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
    ros::Subscriber subMode = nh.subscribe("/cmd_mode", 1000, &Custom::modeCallback, &custom);
    ros::Subscriber subVel = nh.subscribe("/cmd_vel", 1000, &Custom::velCallback, &custom);
    ros::Subscriber subPose = nh.subscribe("/cmd_pose", 1000, &Custom::poseCallback, &custom);

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
