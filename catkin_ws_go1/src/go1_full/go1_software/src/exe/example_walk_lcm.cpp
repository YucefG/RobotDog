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
    void RobotControl();

    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    unitree_legged_msgs::HighCmd SendHighROS;
    unitree_legged_msgs::HighState RecvHighROS;

    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01
};


void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{  
    udp.Send();
}

void Custom::RobotControl() 
{
    motiontime += 2;
    udp.GetRecv(state);

    // reading the high level state does not currently work
    //roslcm.Get(RecvHighLCM);
    // RecvHighROS = ToRos(state);
    // std::cout << "\nstate levelFlag" << state.levelFlag << std::endl;
    // printf("%d   %f\n", motiontime, state.forwardSpeed);
    // printf("%f %f \n", state.imu.quaternion[0], state.imu.quaternion[1]);
    // printf("%f %f \n", state.imu.rpy[1], state.imu.rpy[2]);
    // printf("%f %f %f %f %f\n", state.imu.rpy[1], state.imu.rpy[2], state.position[0], state.position[1], state.velocity[0]);

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

    /* These are the supposed options (HighCmd.msg), note that not everything works. 
    uint8 levelFlag
    uint16 commVersion              # Old version Aliengo does not have
    uint16 robotID                  # Old version Aliengo does not have
    uint32 SN                       # Old version Aliengo does not have
    uint8 bandWidth                 # Old version Aliengo does not have
    uint8 mode                      # 0. idle, default stand  
                                    # 1. force stand (controlled by dBodyHeight + ypr)
                                    # 2. target velocity walking (controlled by velocity + yawSpeed)
                                    # 3. target position walking (controlled by position + ypr[0])
                                    # 4. path mode walking (reserve for future release)
                                    # 5. position stand down. 
                                    # 6. position stand up 
                                    # 7. damping mode 
                                    # 8. recovery stand
                                    # 9. backflip
                                    # 10. jumpYaw
                                    # 11. straightHand
                                    # 12. dance1
                                    # 13. dance2
                                    # 14. two leg stand
    uint8 gaitType                  # 0.idle  1.trot  2.trot running  3.climb stair
    uint8 speedLevel                # 0. default low speed. 1. medium speed 2. high speed. during walking, only respond MODE 3
    float32 footRaiseHeight         # (unit: m, default: 0.08m), foot up height while walking
    float32 bodyHeight              # (unit: m, default: 0.28m),
    float32[2] postion              # (unit: m), desired position in inertial frame
    float32[3] euler                # (unit: rad), roll pitch yaw in stand mode
    float32[2] velocity             # (unit: m/s), forwardSpeed, sideSpeed in body frame
    float32 yawSpeed                # (unit: rad/s), rotateSpeed in body frame
    BmsCmd bms
    LED[4] led
    uint8[40] wirelessRemote
    uint32 reserve                  # Old version Aliengo does not have
    int32 crc
    */

    // roll 
    if(motiontime > 0 && motiontime < 1000){
        cmd.mode = 1;
        cmd.euler[0] = -0.3;
            std::cout<<"roll angle given to udp: "<<cmd.euler[0]<<std::endl;
        std::cout<<"pitch angle given to udp: "<<cmd.euler[1]<<std::endl;
    }
    if(motiontime > 1000 && motiontime < 2000){
        cmd.mode = 1;
        cmd.euler[0] = 0.3;
            std::cout<<"roll angle given to udp: "<<cmd.euler[0]<<std::endl;
        std::cout<<"pitch angle given to udp: "<<cmd.euler[1]<<std::endl;
    }

    // pitch
    if(motiontime > 2000 && motiontime < 3000){
        cmd.mode = 1;
        cmd.euler[1] = -0.2;
                std::cout<<"roll angle given to udp: "<<cmd.euler[0]<<std::endl;
        std::cout<<"pitch angle given to udp: "<<cmd.euler[1]<<std::endl;
    }
    if(motiontime > 3000 && motiontime < 4000){
        cmd.mode = 1;
        cmd.euler[1] = 0.2;
                std::cout<<"roll angle given to udp: "<<cmd.euler[0]<<std::endl;
        std::cout<<"pitch angle given to udp: "<<cmd.euler[1]<<std::endl;
    }

    //yaw 
    if(motiontime > 4000 && motiontime < 5000){
        cmd.mode = 1;
        cmd.euler[2] = -0.2;
                std::cout<<"roll angle given to udp: "<<cmd.euler[0]<<std::endl;
        std::cout<<"pitch angle given to udp: "<<cmd.euler[1]<<std::endl;
    }
    if(motiontime > 5000 && motiontime < 6000){
        cmd.mode = 1;
        cmd.euler[2] = 0.2;
                std::cout<<"roll angle given to udp: "<<cmd.euler[0]<<std::endl;
        std::cout<<"pitch angle given to udp: "<<cmd.euler[1]<<std::endl;
    }

    // move forward
    if(motiontime > 7000 && motiontime < 10000){
        cmd.mode = 2;
        cmd.gaitType = 1;
        cmd.velocity[0] = 0.2f; // -1  ~ +1
        //cmd.bodyHeight = 0.1;
    }

    // move backward
    if(motiontime > 10000 && motiontime < 13000){
        cmd.mode = 2;
        cmd.gaitType = 1;
        cmd.velocity[0] = -0.2f; // -1  ~ +1
        //cmd.bodyHeight = 0.1;
    }

    // move left
    if(motiontime > 13000 && motiontime < 16000){
        cmd.mode = 2;
        cmd.gaitType = 1;
        cmd.velocity[1] = 0.2f; 
    }
    // move right
    if(motiontime > 16000 && motiontime < 19000){
        cmd.mode = 2;
        cmd.gaitType = 1;
        cmd.velocity[1] = -0.2f; 
    }

    udp.SetSend(cmd);
}

int main(void) 
{
    std::cout << "Communication level is set to HIGH-level." << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    Custom custom(HIGHLEVEL);
    InitEnvironment();
    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    while(1){
        sleep(10);
    };

    return 0; 
}