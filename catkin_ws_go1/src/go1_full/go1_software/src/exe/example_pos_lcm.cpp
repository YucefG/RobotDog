/*****************************************************************
 Test position example, read data with ROS, setup like Gazebo
******************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>

#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>

#include "convert.h"

#include "ros/ros.h"
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "body.h" //../unitree_ros/unitree_controller/include/
#include "../include/CurrentState.h"
#include "../../include/OrientationEstimator.h"
#include "../../include/PositionVelocityEstimator.h"
#include "../../include/ContactEstimator.h"
#include "../../include/LegController.h"
#include "../../include/ControlFSMData.h"
#include "../../CPG/HopfPolar.h"

using namespace std;
using namespace UNITREE_LEGGED_SDK;
using namespace unitree_model;

class Custom
{
public:
    Custom(uint8_t level): safe(LeggedType::Go1), udp(level), listen_publish_obj("go1") {
        udp.InitCmdData(lowCmd);
    }
    void init();
    void UDPRecv();
    void UDPSend();
    void RobotControl();

    Safety safe;
    UDP udp;
    LowCmd lowCmd = {0};
    LowState lowState = {0};
    unitree_legged_msgs::LowCmd SendLowROS;
    unitree_legged_msgs::LowState RecvLowROS;

    float qInit[3]={0};
    float qDes[3]={0};
    float sin_mid_q[3] = {0.0, 1.2, -2.0};
    float Kp[3] = {0};  
    float Kd[3] = {0};
    double time_consume = 0;
    int rate_count = 0;
    int sin_count = 0;
    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01

    HopfPolar cpg;

    Quadruped quad ;
    StateEstimate stateEstimate;
    LegController* legController;
    StateEstimatorContainer* stateEstimator;
    ControlFSMData* _controlData;
    CurrentState listen_publish_obj;

    // ros
    ros::NodeHandle n;
    // ros::Publisher lowState_pub; //for rviz visualization
    // ros::ServiceClient set_model_state_serv;
    // ros::Rate loop_rate(1000);
};

void Custom::init()
{  
    string robot_name = "go1";
    // the following nodes have been initialized by "gazebo.launch"
    // lowState_pub = n.advertise<unitree_legged_msgs::LowState>("/" + robot_name + "_gazebo/lowState/state", 1);
    // servo_pub[0] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_hip_controller/command", 1);
    // servo_pub[1] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_thigh_controller/command", 1);
    // servo_pub[2] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_calf_controller/command", 1);
    // servo_pub[3] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_hip_controller/command", 1);
    // servo_pub[4] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_thigh_controller/command", 1);
    // servo_pub[5] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_calf_controller/command", 1);
    // servo_pub[6] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_hip_controller/command", 1);
    // servo_pub[7] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_thigh_controller/command", 1);
    // servo_pub[8] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_calf_controller/command", 1);
    // servo_pub[9] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_hip_controller/command", 1);
    // servo_pub[10] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_thigh_controller/command", 1);
    // servo_pub[11] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_calf_controller/command", 1);

    std::cout << "set quadruped" << std::endl;
    quad.setQuadruped(1); // 1 for Go1, 2 for A1

    // initialize new leg controller and state estimate object
    
    std::cout << "start Leg Controller" << std::endl;
    legController = new LegController(quad);
    legController->zeroCommand();
    // sendServoCmd();

    std::cout << "start state estimate" << std::endl;
    stateEstimator = new StateEstimatorContainer(
                       &RecvLowROS.imu, legController->data,&stateEstimate);
    // using sensors 
    stateEstimator->addEstimator<ContactEstimator>();
    stateEstimator->addEstimator<VectorNavOrientationEstimator>();
    stateEstimator->addEstimator<TunedKFPositionVelocityEstimator>();

    // initialize FSMData
    std::cout << "start controlFSMData" << std::endl;
    _controlData = new ControlFSMData;
    _controlData->_quadruped = &quad;
    _controlData->_stateEstimator = stateEstimator;
    _controlData->_legController = legController;

    
    cpg.init();
    std::cout << "CPG started" << std::endl;
}

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
    udp.GetRecv(lowState);
    RecvLowROS = ToRos(lowState);
    // update data
    _controlData->_legController->updateData();
    _controlData->_stateEstimator->run();

    printf("%d  %f\n", motiontime, lowState.motorState[FR_2].q);
    printf("%d  %f\n", motiontime, lowState.imu.quaternion[2]); 

    // gravity compensation
    lowCmd.motorCmd[FR_0].tau = -0.65f;
    lowCmd.motorCmd[FL_0].tau = +0.65f;
    lowCmd.motorCmd[RR_0].tau = -0.65f;
    lowCmd.motorCmd[RL_0].tau = +0.65f;

    // if( motiontime >= 100){
    if( motiontime >= 0){
        // first, get record initial position
        // if( motiontime >= 100 && motiontime < 500){
        if( motiontime >= 0 && motiontime < 10){
            qInit[0] = lowState.motorState[FR_0].q;
            qInit[1] = lowState.motorState[FR_1].q;
            qInit[2] = lowState.motorState[FR_2].q;
        }
        // second, move to the origin point of a sine movement with Kp Kd
        // if( motiontime >= 500 && motiontime < 1500){
        if( motiontime >= 10 && motiontime < 400){
            rate_count++;
            double rate = rate_count/200.0;                       // needs count to 200
            Kp[0] = 5.0; Kp[1] = 5.0; Kp[2] = 5.0; 
            Kd[0] = 1.0; Kd[1] = 1.0; Kd[2] = 1.0;
            
            qDes[0] = jointLinearInterpolation(qInit[0], sin_mid_q[0], rate);
            qDes[1] = jointLinearInterpolation(qInit[1], sin_mid_q[1], rate);
            qDes[2] = jointLinearInterpolation(qInit[2], sin_mid_q[2], rate);
        }
        double sin_joint1, sin_joint2;
        // last, do sine wave
        if( motiontime >= 400){
            sin_count++;
            sin_joint1 = 0.6 * sin(3*M_PI*sin_count/1000.0);
            sin_joint2 = -0.6 * sin(1.8*M_PI*sin_count/1000.0);
            qDes[0] = sin_mid_q[0];
            qDes[1] = sin_mid_q[1];
            qDes[2] = sin_mid_q[2] + sin_joint2;
            // qDes[2] = sin_mid_q[2];
        }

        lowCmd.motorCmd[FR_0].q = qDes[0];
        lowCmd.motorCmd[FR_0].dq = 0;
        lowCmd.motorCmd[FR_0].Kp = Kp[0];
        lowCmd.motorCmd[FR_0].Kd = Kd[0];
        lowCmd.motorCmd[FR_0].tau = -0.65f;

        lowCmd.motorCmd[FR_1].q = qDes[1];
        lowCmd.motorCmd[FR_1].dq = 0;
        lowCmd.motorCmd[FR_1].Kp = Kp[1];
        lowCmd.motorCmd[FR_1].Kd = Kd[1];
        lowCmd.motorCmd[FR_1].tau = 0.0f;

        lowCmd.motorCmd[FR_2].q =  qDes[2];
        lowCmd.motorCmd[FR_2].dq = 0;
        lowCmd.motorCmd[FR_2].Kp = Kp[2];
        lowCmd.motorCmd[FR_2].Kd = Kd[2];
        lowCmd.motorCmd[FR_2].tau = 0.0f;

    }


    // test we can compute this here 
    double sideSign[4] = {-1, 1, -1, 1};
    double foot_y = 0.08;
    // for (int i=0; i<4; i++){
    //     // desired foot position and corresponding joint angles
    //     Vec3<double> pDes, qDes2; //, tau;
    //     pDes << 0, sideSign[i] * foot_y, -0.3;
    //     computeInverseKinematics(_controlData->_legController->_quadruped, pDes, i, &qDes2);
    //     std::cout << i << " " << qDes2[0] << " " << qDes2[1] << " " << qDes2[2] << std::endl;
    // }

    if(motiontime > 10){
        safe.PositionLimit(lowCmd);
        int res1 = safe.PowerProtect(lowCmd, lowState, 1);
        // You can uncomment it for position protection
        // int res2 = safe.PositionProtect(cmd, state, 0.087);
        if(res1 < 0) exit(-1);
    }

    udp.SetSend(lowCmd);

}


int main(int argc, char **argv)
{
    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    
    ros::init(argc, argv, "go1_test_setup");

    // HopfPolar cpg;
    Custom custom(LOWLEVEL);
    custom.init();
    // InitEnvironment();
    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    // LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    // LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, boost::bind(&Custom::UDPRecv,      &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    while(1){
        sleep(10);
    };

    return 0; 
}
