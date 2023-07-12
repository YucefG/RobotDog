/*!
 * @file CurrentState.h
 * @brief function as FSM of Aliengo robot
 * call back all data of current state
 * 0 /1 /2 /3
 * FR/FL/RR/RL
 */ 
#ifndef CURRENT_STATE_H
#define CURRENT_STATE_H

#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"
#include "unitree_legged_msgs/IMU.h"
#include "unitree_legged_msgs/CheaterState.h"
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <string.h>
#include <math.h>
#include "body.h"

using namespace std;
using namespace unitree_model;


class CurrentState {
    public:
        CurrentState(string rname);
        void cheaterCallback(const gazebo_msgs::ModelStates& msg);
        // void imuCallback(const unitree_legged_msgs::IMU& msg);
        void imuCallback(const sensor_msgs::Imu& msg);
        void FRhipCallback(const unitree_legged_msgs::MotorState& msg);
        void FRthighCallback(const unitree_legged_msgs::MotorState& msg);
        void FRcalfCallback(const unitree_legged_msgs::MotorState& msg);
        void FLhipCallback(const unitree_legged_msgs::MotorState& msg);
        void FLthighCallback(const unitree_legged_msgs::MotorState& msg);
        void FLcalfCallback(const unitree_legged_msgs::MotorState& msg);
        void RRhipCallback(const unitree_legged_msgs::MotorState& msg);
        void RRthighCallback(const unitree_legged_msgs::MotorState& msg);
        void RRcalfCallback(const unitree_legged_msgs::MotorState& msg);
        void RLhipCallback(const unitree_legged_msgs::MotorState& msg);
        void RLthighCallback(const unitree_legged_msgs::MotorState& msg);
        void RLcalfCallback(const unitree_legged_msgs::MotorState& msg);
        void FRfootCallback(const geometry_msgs::WrenchStamped& msg);
        void FLfootCallback(const geometry_msgs::WrenchStamped& msg);
        void RRfootCallback(const geometry_msgs::WrenchStamped& msg);
        void RLfootCallback(const geometry_msgs::WrenchStamped& msg);

        double nominal_height = 0.32;

    private:
        ros::NodeHandle nm;
        ros::Subscriber servo_sub[12], footForce_sub[4], imu_sub, state_sub;
        string robot_name;

};
#endif
