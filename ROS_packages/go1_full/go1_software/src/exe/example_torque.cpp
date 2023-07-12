/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    Custom(uint8_t level): safe(LeggedType::Go1), udp(level){
        udp.InitCmdData(cmd);
    }
    void UDPSend();
    void UDPRecv();
    void RobotControl();

    Safety safe;
    UDP udp;
    LowCmd cmd = {0};
    LowState state = {0};
    int motiontime = 0;
    float dt = 0.001;     // 0.001~0.01
    float qInit[3]={0};
    float qDes[3]={0};
    int rate_count = 0;
    float Kp[3] = {0};  
    float Kd[3] = {0};
};

void Custom::UDPRecv()
{ 
    udp.Recv();
}

void Custom::UDPSend()
{  
    udp.Send();
}

double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos*(1-rate) + targetPos*rate;
    return p;
}

void Custom::RobotControl() 
{
    motiontime++;
    udp.GetRecv(state);
    // printf("%d\n", motiontime);
    // gravity compensation
    cmd.motorCmd[FR_0].tau = -0.65f;
    cmd.motorCmd[FL_0].tau = +0.65f;
    cmd.motorCmd[RR_0].tau = -0.65f;
    cmd.motorCmd[RL_0].tau = +0.65f;

    // first, get record initial position
    // if( motiontime >= 100 && motiontime < 500){
    if( motiontime >= 0 && motiontime < 10){
        qInit[0] = state.motorState[FR_0].q;
        qInit[1] = state.motorState[FR_1].q;
        qInit[2] = state.motorState[FR_2].q;
    }
    // second, move to the origin point of a sine movement with Kp Kd
    // if( motiontime >= 500 && motiontime < 1500){
    if( motiontime >= 10 && motiontime < 2000){
        rate_count++;
        double rate = rate_count/200.0;                       // needs count to 200
        Kp[0] = 5.0; Kp[1] = 5.0; Kp[2] = 5.0; 
        Kd[0] = 1.0; Kd[1] = 1.0; Kd[2] = 1.0;
        
        qDes[0] = jointLinearInterpolation(qInit[0], 0, rate);
        qDes[1] = jointLinearInterpolation(qInit[1], 0, rate);
        qDes[2] = jointLinearInterpolation(qInit[2], -M_PI/2, rate);

        cmd.motorCmd[FR_0].q = qDes[0];
        cmd.motorCmd[FR_0].dq = 0;
        cmd.motorCmd[FR_0].Kp = Kp[0];
        cmd.motorCmd[FR_0].Kd = Kd[0];
        cmd.motorCmd[FR_0].tau = -0.65f;

        cmd.motorCmd[FR_1].q = qDes[1];
        cmd.motorCmd[FR_1].dq = 0;
        cmd.motorCmd[FR_1].Kp = Kp[1];
        cmd.motorCmd[FR_1].Kd = Kd[1];
        cmd.motorCmd[FR_1].tau = 0.0f;

        cmd.motorCmd[FR_2].q =  qDes[2];
        cmd.motorCmd[FR_2].dq = 0;
        cmd.motorCmd[FR_2].Kp = Kp[2];
        cmd.motorCmd[FR_2].Kd = Kd[2];
        cmd.motorCmd[FR_2].tau = 0.0f;
    }

    /* gains that worked well:
        35 0.5
    */
    if( motiontime >= 2000){
        float des_pos = 0.5 * std::sin(1.5*M_PI*(motiontime-2000) * dt);
        float Kp = 35.0f;
        float Kd = 0.5f;
        float torque = Kp * (des_pos - state.motorState[FR_1].q) + Kd * (0 - state.motorState[FR_1].dq);
        if(torque > 5.0f){
            torque = 5.0f;
            printf("torque too large %f\n", torque);
        }
        if(torque < -5.0f) {
            torque = -5.0f;
            printf("torque too small %f\n", torque);
        }

        cmd.motorCmd[FR_1].q = PosStopF;
        cmd.motorCmd[FR_1].dq = VelStopF;
        cmd.motorCmd[FR_1].Kp = 0;
        cmd.motorCmd[FR_1].Kd = 0;
        cmd.motorCmd[FR_1].tau = torque;


        cmd.motorCmd[FR_2].q = PosStopF;
        cmd.motorCmd[FR_2].dq = VelStopF;
        cmd.motorCmd[FR_2].Kp = 0;
        cmd.motorCmd[FR_2].Kd = 0;
        cmd.motorCmd[FR_2].tau = Kp * (-M_PI/2 - state.motorState[FR_2].q) + Kd * (0 - state.motorState[FR_2].dq);
    }
    int res = safe.PowerProtect(cmd, state, 1);
    if(res < 0) exit(-1);

    udp.SetSend(cmd);
}

int main(void)
{
    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    Custom custom(LOWLEVEL);
    // InitEnvironment();
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
